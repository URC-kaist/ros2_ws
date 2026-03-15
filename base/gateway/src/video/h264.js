'use strict'

const ACCESS_UNIT_BOUNDARY_TYPES = new Set([6, 7, 8, 9, 10, 11, 12])

function findStartCodeLength(buffer, offset) {
  if (offset + 3 > buffer.length) return 0
  if (buffer[offset] !== 0 || buffer[offset + 1] !== 0) return 0
  if (buffer[offset + 2] === 1) return 3
  if (offset + 4 <= buffer.length && buffer[offset + 2] === 0 && buffer[offset + 3] === 1) {
    return 4
  }
  return 0
}

function findStartCodeOffsets(buffer) {
  const offsets = []
  for (let index = 0; index < buffer.length - 2; index += 1) {
    const length = findStartCodeLength(buffer, index)
    if (length > 0) {
      offsets.push(index)
      index += length - 1
    }
  }
  return offsets
}

function splitAnnexBNalus(buffer, allowTrailing = false) {
  const starts = findStartCodeOffsets(buffer)
  if (starts.length === 0) {
    return {
      nals: [],
      remainder: buffer,
    }
  }

  const firstStart = starts[0]
  const nals = []

  for (let index = 0; index < starts.length - 1; index += 1) {
    nals.push(buffer.slice(starts[index], starts[index + 1]))
  }

  const lastStart = starts[starts.length - 1]
  if (allowTrailing) {
    nals.push(buffer.slice(lastStart))
    return {
      nals,
      remainder: Buffer.alloc(0),
    }
  }

  return {
    nals,
    remainder: buffer.slice(lastStart),
  }
}

function stripStartCode(nal) {
  const length = findStartCodeLength(nal, 0)
  return length > 0 ? nal.slice(length) : nal
}

function getNalType(nal) {
  const body = stripStartCode(nal)
  if (body.length === 0) return null
  return body[0] & 0x1f
}

function isVclNalType(type) {
  return type != null && type >= 1 && type <= 5
}

function removeEmulationPreventionBytes(buffer) {
  const bytes = []
  for (let index = 0; index < buffer.length; index += 1) {
    if (
      index >= 2 &&
      buffer[index] === 0x03 &&
      buffer[index - 1] === 0x00 &&
      buffer[index - 2] === 0x00
    ) {
      continue
    }
    bytes.push(buffer[index])
  }
  return Buffer.from(bytes)
}

class BitReader {
  constructor(buffer) {
    this.buffer = buffer
    this.bitOffset = 0
  }

  readBit() {
    if (this.bitOffset >= this.buffer.length * 8) {
      throw new Error('bitstream underflow')
    }
    const byteOffset = Math.floor(this.bitOffset / 8)
    const shift = 7 - (this.bitOffset % 8)
    const value = (this.buffer[byteOffset] >> shift) & 0x01
    this.bitOffset += 1
    return value
  }

  readUnsignedExpGolomb() {
    let leadingZeros = 0
    while (this.readBit() === 0) {
      leadingZeros += 1
    }

    let value = 1
    for (let index = 0; index < leadingZeros; index += 1) {
      value = (value << 1) | this.readBit()
    }
    return value - 1
  }
}

function getFirstMbInSlice(nal) {
  const body = stripStartCode(nal)
  if (body.length < 2) return null

  try {
    const rbsp = removeEmulationPreventionBytes(body.slice(1))
    const reader = new BitReader(rbsp)
    return reader.readUnsignedExpGolomb()
  } catch {
    return null
  }
}

function inspectNalUnit(nal) {
  const type = getNalType(nal)
  const body = stripStartCode(nal)
  return {
    nal,
    type,
    isVcl: isVclNalType(type),
    firstMbInSlice: isVclNalType(type) ? getFirstMbInSlice(nal) : null,
    raw: body,
  }
}

function deriveCodecString(spsNal) {
  if (!spsNal || spsNal.length < 4) return null
  const profile = spsNal[1].toString(16).padStart(2, '0').toUpperCase()
  const constraints = spsNal[2].toString(16).padStart(2, '0').toUpperCase()
  const level = spsNal[3].toString(16).padStart(2, '0').toUpperCase()
  return `avc1.${profile}${constraints}${level}`
}

function classifyAccessUnit(nals) {
  const inspected = nals.map(inspectNalUnit)
  let sps = null
  let pps = null
  let key = false
  let delta = false
  let codec = null

  for (const nal of inspected) {
    if (nal.type === 7) {
      sps = nal.raw
      codec = deriveCodecString(nal.raw)
    } else if (nal.type === 8) {
      pps = nal.raw
    } else if (nal.type === 5) {
      key = true
    } else if (nal.type === 1) {
      delta = true
    }
  }

  if (key) {
    delta = false
  }

  return {
    codec,
    delta,
    key,
    payload: Buffer.concat(nals),
    pps,
    sps,
  }
}

class AnnexBAccessUnitParser {
  constructor() {
    this.pending = Buffer.alloc(0)
    this.currentNals = []
    this.currentHasVcl = false
  }

  push(chunk) {
    if (!chunk || chunk.length === 0) return []
    this.pending = Buffer.concat([this.pending, chunk])
    return this.consume(false)
  }

  flush() {
    const emitted = this.consume(true)
    if (this.currentNals.length === 0) {
      return emitted
    }
    emitted.push(this.emitCurrent())
    return emitted
  }

  consume(allowTrailing) {
    const emitted = []
    const { nals, remainder } = splitAnnexBNalus(this.pending, allowTrailing)
    this.pending = remainder

    for (const nal of nals) {
      const info = inspectNalUnit(nal)
      if (this.shouldStartNewAccessUnit(info)) {
        emitted.push(this.emitCurrent())
      }
      this.currentNals.push(nal)
      if (info.isVcl) {
        this.currentHasVcl = true
      }
    }

    return emitted
  }

  shouldStartNewAccessUnit(nalInfo) {
    if (this.currentNals.length === 0) return false
    if (nalInfo.type === 9) return true
    if (nalInfo.isVcl) {
      return this.currentHasVcl && nalInfo.firstMbInSlice === 0
    }
    return this.currentHasVcl && ACCESS_UNIT_BOUNDARY_TYPES.has(nalInfo.type)
  }

  emitCurrent() {
    const current = this.currentNals
    this.currentNals = []
    this.currentHasVcl = false
    return classifyAccessUnit(current)
  }
}

module.exports = {
  AnnexBAccessUnitParser,
  classifyAccessUnit,
  deriveCodecString,
  getFirstMbInSlice,
  getNalType,
  splitAnnexBNalus,
  stripStartCode,
}
