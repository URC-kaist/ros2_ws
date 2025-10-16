# sender.py
import argparse
import asyncio
import functools
import json
import threading
import sys

import gi
gi.require_version('Gst', '1.0')
gi.require_version('GstWebRTC', '1.0')
gi.require_version('GstSdp', '1.0')
gi.require_version('GstVideo', '1.0')

import websockets
from gi.repository import Gst, GstWebRTC, GstSdp, GstVideo, GLib


def _patch_asyncio_loop_kwargs():
    if sys.version_info < (3, 10):
        return

    def _wrap(cls):
        orig_init = cls.__init__

        class LoopCompat(cls):
            __doc__ = cls.__doc__
            _loop_compat_wrapped = True

            def __init__(self, *args, **kwargs):
                kwargs.pop("loop", None)
                orig_init(self, *args, **kwargs)

        LoopCompat.__name__ = cls.__name__
        LoopCompat.__qualname__ = cls.__qualname__
        LoopCompat.__module__ = cls.__module__
        return LoopCompat

    for attr in (
        "Lock",
        "Event",
        "Condition",
        "Semaphore",
        "BoundedSemaphore",
        "Queue",
        "LifoQueue",
        "PriorityQueue",
    ):
        cls = getattr(asyncio, attr, None)
        if cls is None or getattr(cls, "_loop_compat_wrapped", False):
            continue
        setattr(asyncio, attr, _wrap(cls))

    def _wrap_async(func):
        if getattr(func, "_loop_compat_wrapped", False):
            return func

        @functools.wraps(func)
        async def wrapper(*args, **kwargs):
            kwargs.pop("loop", None)
            return await func(*args, **kwargs)

        wrapper._loop_compat_wrapped = True
        return wrapper

    for fname in ("sleep", "wait", "wait_for"):
        func = getattr(asyncio, fname, None)
        if func is None:
            continue
        setattr(asyncio, fname, _wrap_async(func))


_patch_asyncio_loop_kwargs()

class Sender:
    def __init__(self, ws_url, video_device, use_pulse):
        self.ws_url = ws_url
        self.video_device = video_device
        self.use_pulse = use_pulse
        self.ws = None
        self.loop = None
        self.mainloop = None
        self.pipeline = None
        self.webrtc = None
        self._pending_signaling = []
        self.vp8enc = None
        self.rtpvp8pay = None

    # ---------- GStreamer setup ----------
    def build_pipeline(self):
        Gst.init(None)

        self.pipeline = Gst.Pipeline.new("webrtc-pipeline")
        self.webrtc = Gst.ElementFactory.make("webrtcbin", "webrtcbin")
        if not self.webrtc:
            raise RuntimeError("webrtcbin not found. Install gstreamer1.0-plugins-bad.")
        self.webrtc.set_property("stun-server", "stun://stun.l.google.com:19302")
        # Optional: self.webrtc.set_property("bundle-policy", 3)  # max-bundle

        # VIDEO: v4l2src → convert/rate → VP8 → RTP → caps → queue → webrtcbin
        if self.video_device == "test":
            vsrc = Gst.ElementFactory.make("videotestsrc", "vsrc")
            vsrc.set_property("is-live", True)
            vsrc.set_property("pattern", 18)  # ball pattern
        else:
            vsrc = Gst.ElementFactory.make("v4l2src", "vsrc")
            vsrc.set_property("device", self.video_device)

        vconv = Gst.ElementFactory.make("videoconvert", "vconv")
        vrate = Gst.ElementFactory.make("videorate", "vrate")
        vcaps = Gst.ElementFactory.make("capsfilter", "vcaps")
        vcaps.set_property("caps", Gst.Caps.from_string("video/x-raw,width=1280,height=720,framerate=30/1"))

        vp8enc = Gst.ElementFactory.make("vp8enc", "vp8enc")
        vp8enc.set_property("deadline", 1)   # low-latency
        vp8enc.set_property("cpu-used", 8)
        vp8enc.set_property("keyframe-max-dist", 30)  # emit keyframe roughly every second (30 fps)
        self.vp8enc = vp8enc

        rtpvp8pay = Gst.ElementFactory.make("rtpvp8pay", "rtpvp8pay")
        rtpvp8pay.set_property("pt", 96)
        self.rtpvp8pay = rtpvp8pay

        vcaprtp = Gst.ElementFactory.make("capsfilter", "vcaprtp")
        vcaprtp.set_property("caps", Gst.Caps.from_string(
            "application/x-rtp,media=video,encoding-name=VP8,payload=96"
        ))

        qv = Gst.ElementFactory.make("queue", "qv")

        # AUDIO: alsasrc|pulsesrc → convert/resample → opus → RTP → caps → queue → webrtcbin
        asrc = Gst.ElementFactory.make("pulsesrc" if self.use_pulse else "alsasrc", "asrc")
        aconv = Gst.ElementFactory.make("audioconvert", "aconv")
        ares = Gst.ElementFactory.make("audioresample", "ares")
        opus = Gst.ElementFactory.make("opusenc", "opus")
        opus.set_property("bitrate", 64000)
        rtopus = Gst.ElementFactory.make("rtpopuspay", "rtopus")
        rtopus.set_property("pt", 97)

        acaprtp = Gst.ElementFactory.make("capsfilter", "acaprtp")
        acaprtp.set_property("caps", Gst.Caps.from_string(
            "application/x-rtp,media=audio,encoding-name=OPUS,payload=97"
        ))

        qa = Gst.ElementFactory.make("queue", "qa")

        for e in (vsrc, vconv, vrate, vcaps, vp8enc, rtpvp8pay, vcaprtp, qv,
                  asrc, aconv, ares, opus, rtopus, acaprtp, qa, self.webrtc):
            self.pipeline.add(e)

        self._link_elements(
            (vsrc, vconv, vrate, vcaps, vp8enc, rtpvp8pay, vcaprtp, qv),
            "video"
        )

        self._link_elements(
            (asrc, aconv, ares, opus, rtopus, acaprtp, qa),
            "audio"
        )

        # Link RTP → webrtcbin via request sink pads
        v_srcpad = qv.get_static_pad("src")
        a_srcpad = qa.get_static_pad("src")
        v_sinkpad = self.webrtc.get_request_pad("sink_%u")
        a_sinkpad = self.webrtc.get_request_pad("sink_%u")
        if v_srcpad.link(v_sinkpad) != Gst.PadLinkReturn.OK:
            raise RuntimeError("Video → webrtcbin link failed")
        if a_srcpad.link(a_sinkpad) != Gst.PadLinkReturn.OK:
            raise RuntimeError("Audio → webrtcbin link failed")

        # ICE candidates from GStreamer → browser
        self.webrtc.connect("on-ice-candidate", self._on_ice_candidate)

        # Bus logging
        bus = self.pipeline.get_bus()
        bus.add_signal_watch()
        bus.connect("message::error", self._on_bus_error)
        bus.connect("message::warning", self._on_bus_warning)

    def start_gst(self):
        self.build_pipeline()
        ret = self.pipeline.set_state(Gst.State.PLAYING)
        ret_name = getattr(ret, "value_nick", None) or getattr(ret, "value_name", None) or str(ret)
        print(f"[GST] pipeline PLAYING: {ret_name}")
        self.mainloop = GLib.MainLoop()
        threading.Thread(target=self.mainloop.run, daemon=True).start()

    # ---------- GStreamer callbacks ----------
    def _on_bus_error(self, bus, msg):
        err, dbg = msg.parse_error()
        print(f"[GST][ERROR] {err} debug:{dbg}")

    def _on_bus_warning(self, bus, msg):
        w, dbg = msg.parse_warning()
        print(f"[GST][WARN] {w} debug:{dbg}")

    def _on_ice_candidate(self, webrtcbin, mlineindex, candidate):
        # Send to browser via WS (from GLib thread -> asyncio loop)
        payload = {"type": "candidate",
                   "candidate": {"candidate": candidate, "sdpMLineIndex": int(mlineindex)}}
        if self.ws is not None and self.loop is not None:
            asyncio.run_coroutine_threadsafe(self.ws.send(json.dumps(payload)), self.loop)
        else:
            self._pending_signaling.append(payload)

    # Called inside GLib thread using GLib.idle_add
    def _handle_offer_glib(self, sdp_str):
        ok, sdp = GstSdp.SDPMessage.new()  # IMPORTANT: this returns (ok, sdp)
        if ok != GstSdp.SDPResult.OK:
            print("[GST] SDPMessage.new failed")
            return False
        # Parse remote offer
        res = GstSdp.sdp_message_parse_buffer(sdp_str.encode("utf-8"), sdp)
        if res != GstSdp.SDPResult.OK:
            print(f"[GST] SDP parse failed: {res}")
            return False
        print("[GST] remote offer parsed")

        offer = GstWebRTC.WebRTCSessionDescription.new(
            GstWebRTC.WebRTCSDPType.OFFER, sdp
        )

        def on_set_remote_description(promise, _):
            print("[GST] remote description set")
            # Once remote set, create our answer
            def on_answer_created(promise2, _):
                reply = promise2.get_reply()
                answer = reply.get_value("answer")
                print("[GST] answer created")
                # Set local
                self.webrtc.emit("set-local-description", answer, Gst.Promise.new())
                # Send answer to browser
                sdp_text = answer.sdp.as_text()
                preview = sdp_text.splitlines()[0:10]
                print("[GST] sending answer preview:")
                for line in preview:
                    print("  ", line)
                self._force_keyframe()
                if self.ws:
                    asyncio.run_coroutine_threadsafe(
                        self.ws.send(json.dumps({"type": "answer", "sdp": sdp_text})),
                        self.loop
                    )

            self.webrtc.emit("create-answer", None,
                             Gst.Promise.new_with_change_func(on_answer_created, None))

        self.webrtc.emit("set-remote-description", offer,
                         Gst.Promise.new_with_change_func(on_set_remote_description, None))
        return False  # remove idle

    def _add_ice_glib(self, idx, cand):
        try:
            self.webrtc.emit("add-ice-candidate", int(idx), cand)
            print(f"[GST] added remote ICE (mline {idx})")
        except Exception as e:
            print(f"[GST] add-ice-candidate failed: {e}")
        return False

    # ---------- WebSocket (asyncio) ----------
    async def run_ws(self):
        self.loop = asyncio.get_running_loop()
        async with websockets.connect(self.ws_url, ping_interval=20) as ws:
            self.ws = ws
            print("[sender] connected to signaling")

            # flush any messages gathered before websocket ready (e.g., local ICE)
            for payload in self._pending_signaling:
                await self.ws.send(json.dumps(payload))
            self._pending_signaling.clear()

            async for raw in ws:
                msg = json.loads(raw)
                print(f"[sender] received {msg.get('type')}")
                t = msg.get("type")
                if t == "offer":
                    # Schedule SDP handling on GLib thread
                    GLib.idle_add(self._handle_offer_glib, msg["sdp"])
                elif t == "candidate":
                    c = msg.get("candidate")
                    if isinstance(c, dict):
                        cand = c.get("candidate")
                        idx = c.get("sdpMLineIndex", 0)
                    else:
                        cand = c
                        idx = msg.get("sdpMLineIndex", 0)
                    GLib.idle_add(self._add_ice_glib, idx, cand)

    def start(self):
        self.start_gst()
        return self.run_ws()

    # ---------- helpers ----------
    def _force_keyframe(self):
        if not self.vp8enc:
            return
        target = None
        if self.rtpvp8pay:
            target = self.rtpvp8pay.get_static_pad("src")
        if not target and self.vp8enc:
            target = self.vp8enc.get_static_pad("src")
        if not target:
            queue = self.pipeline.get_by_name("qv") if self.pipeline else None
            if queue:
                target = queue.get_static_pad("src")
        if not target:
            return
        event = GstVideo.video_event_new_upstream_force_key_unit(
            Gst.CLOCK_TIME_NONE, True, 0
        )
        if target.send_event(event):
            print("[GST] forced keyframe request sent")
        else:
            print("[GST] failed to force keyframe")

    def _link_elements(self, elements, label):
        for left, right in zip(elements[:-1], elements[1:]):
            if not left.link(right):
                lname = left.get_name() if left else "<None>"
                rname = right.get_name() if right else "<None>"
                raise RuntimeError(f"Failed linking {label} chain at {lname} -> {rname}")

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ws", default="ws://127.0.0.1:8080/ws?role=sender",
                    help="Signaling WebSocket URL")
    ap.add_argument("--video", default="/dev/video0",
                    help="V4L2 device path or 'test' to use a synthetic pattern")
    ap.add_argument("--pulse", action="store_true", help="Use pulsesrc instead of alsasrc")
    args = ap.parse_args()

    sender = Sender(args.ws, args.video, args.pulse)
    asyncio.run(sender.start())

if __name__ == "__main__":
    main()
