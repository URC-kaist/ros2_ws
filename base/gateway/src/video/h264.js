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

  readBits(count) {
    let value = 0
    for (let index = 0; index < count; index += 1) {
      value = (value << 1) | this.readBit()
    }
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

  readSignedExpGolomb() {
    const codeNum = this.readUnsignedExpGolomb()
    const magnitude = Math.ceil(codeNum / 2)
    return codeNum % 2 === 0 ? -magnitude : magnitude
  }
}

function skipScalingList(reader, size) {
  let lastScale = 8
  let nextScale = 8

  for (let index = 0; index < size; index += 1) {
    if (nextScale !== 0) {
      const deltaScale = reader.readSignedExpGolomb()
      nextScale = (lastScale + deltaScale + 256) % 256
    }
    lastScale = nextScale === 0 ? lastScale : nextScale
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

function parseSpsDimensions(spsNal) {
  if (!spsNal || spsNal.length < 4) return null

  try {
    const body = stripStartCode(spsNal)
    if (body.length < 4) return null

    const rbsp = removeEmulationPreventionBytes(body.slice(1))
    const reader = new BitReader(rbsp)

    const profileIdc = reader.readBits(8)
    reader.readBits(8) // constraint flags and reserved bits
    reader.readBits(8) // level_idc
    reader.readUnsignedExpGolomb() // seq_parameter_set_id

    let chromaFormatIdc = 1
    if (new Set([44, 83, 86, 100, 110, 118, 122, 128, 134, 135, 138, 139, 244]).has(profileIdc)) {
      chromaFormatIdc = reader.readUnsignedExpGolomb()
      if (chromaFormatIdc === 3) {
        reader.readBit() // separate_colour_plane_flag
      }
      reader.readUnsignedExpGolomb() // bit_depth_luma_minus8
      reader.readUnsignedExpGolomb() // bit_depth_chroma_minus8
      reader.readBit() // qpprime_y_zero_transform_bypass_flag
      const seqScalingMatrixPresentFlag = reader.readBit()
      if (seqScalingMatrixPresentFlag) {
        const scalingCount = chromaFormatIdc !== 3 ? 8 : 12
        for (let index = 0; index < scalingCount; index += 1) {
          const seqScalingListPresentFlag = reader.readBit()
          if (seqScalingListPresentFlag) {
            skipScalingList(reader, index < 6 ? 16 : 64)
          }
        }
      }
    }

    reader.readUnsignedExpGolomb() // log2_max_frame_num_minus4
    const picOrderCntType = reader.readUnsignedExpGolomb()
    if (picOrderCntType === 0) {
      reader.readUnsignedExpGolomb() // log2_max_pic_order_cnt_lsb_minus4
    } else if (picOrderCntType === 1) {
      reader.readBit() // delta_pic_order_always_zero_flag
      reader.readSignedExpGolomb() // offset_for_non_ref_pic
      reader.readSignedExpGolomb() // offset_for_top_to_bottom_field
      const cycleLength = reader.readUnsignedExpGolomb()
      for (let index = 0; index < cycleLength; index += 1) {
        reader.readSignedExpGolomb()
      }
    }

    reader.readUnsignedExpGolomb() // max_num_ref_frames
    reader.readBit() // gaps_in_frame_num_value_allowed_flag
    const picWidthInMbsMinus1 = reader.readUnsignedExpGolomb()
    const picHeightInMapUnitsMinus1 = reader.readUnsignedExpGolomb()
    const frameMbsOnlyFlag = reader.readBit()
    if (frameMbsOnlyFlag === 0) {
      reader.readBit() // mb_adaptive_frame_field_flag
    }
    reader.readBit() // direct_8x8_inference_flag

    const frameCroppingFlag = reader.readBit()
    let frameCropLeftOffset = 0
    let frameCropRightOffset = 0
    let frameCropTopOffset = 0
    let frameCropBottomOffset = 0
    if (frameCroppingFlag) {
      frameCropLeftOffset = reader.readUnsignedExpGolomb()
      frameCropRightOffset = reader.readUnsignedExpGolomb()
      frameCropTopOffset = reader.readUnsignedExpGolomb()
      frameCropBottomOffset = reader.readUnsignedExpGolomb()
    }

    let width = (picWidthInMbsMinus1 + 1) * 16
    let height = (picHeightInMapUnitsMinus1 + 1) * 16 * (2 - frameMbsOnlyFlag)

    let cropUnitX = 1
    let cropUnitY = 2 - frameMbsOnlyFlag
    if (chromaFormatIdc === 1) {
      cropUnitX = 2
      cropUnitY = 2 * (2 - frameMbsOnlyFlag)
    } else if (chromaFormatIdc === 2) {
      cropUnitX = 2
      cropUnitY = 2 - frameMbsOnlyFlag
    } else if (chromaFormatIdc === 3) {
      cropUnitX = 1
      cropUnitY = 2 - frameMbsOnlyFlag
    }

    width -= (frameCropLeftOffset + frameCropRightOffset) * cropUnitX
    height -= (frameCropTopOffset + frameCropBottomOffset) * cropUnitY

    if (width <= 0 || height <= 0) return null
    return { width, height }
  } catch {
    return null
  }
}

function classifyAccessUnit(nals) {
  const inspected = nals.map(inspectNalUnit)
  let sps = null
  let pps = null
  let key = false
  let delta = false
  let codec = null
  let width = null
  let height = null

  for (const nal of inspected) {
    if (nal.type === 7) {
      sps = nal.raw
      codec = deriveCodecString(nal.raw)
      const dimensions = parseSpsDimensions(nal.raw)
      if (dimensions) {
        width = dimensions.width
        height = dimensions.height
      }
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
    height,
    key,
    payload: Buffer.concat(nals),
    pps,
    sps,
    width,
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
  parseSpsDimensions,
  splitAnnexBNalus,
  stripStartCode,
}
