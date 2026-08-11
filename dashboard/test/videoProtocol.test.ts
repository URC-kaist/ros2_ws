import assert from 'node:assert/strict'

import { decodeVideoMessage, VIDEO_MESSAGE_TIMED_CHUNK } from '../src/lib/videoProtocol'

const streamId = new TextEncoder().encode('front')
const payload = new Uint8Array([1, 2, 3])
const buffer = new ArrayBuffer(34 + streamId.length + payload.length)
const view = new DataView(buffer)
view.setUint8(0, VIDEO_MESSAGE_TIMED_CHUNK)
view.setUint16(1, streamId.length, true)
view.setUint8(3, 1)
view.setBigUint64(4, 111n, true)
view.setUint32(12, 42, true)
view.setUint32(16, 90000, true)
view.setUint16(20, 65535, true)
view.setBigUint64(22, 222n, true)
view.setUint32(30, payload.length, true)
new Uint8Array(buffer).set(streamId, 34)
new Uint8Array(buffer).set(payload, 34 + streamId.length)

const decoded = decodeVideoMessage(buffer)
assert.equal(decoded?.kind, 'chunk')
if (decoded?.kind !== 'chunk') throw new Error('timed chunk did not decode')
assert.equal(decoded.timestampUs, 111)
assert.equal(decoded.baseIngestTimestampUs, 222)
assert.deepEqual(decoded.correlation, {
  ssrc: 42,
  rtpTimestamp: 90000,
  markerSequence: 65535,
})

console.log('video protocol timed chunk test passed')
