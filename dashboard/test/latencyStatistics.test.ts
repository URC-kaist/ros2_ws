import assert from 'node:assert/strict'

import { distribution, median, percentile } from '../src/lib/latency/statistics'

assert.equal(median([]), null)
assert.equal(median([3, 1, 2]), 2)
assert.equal(median([4, 1, 3, 2]), 2.5)
assert.equal(percentile([0, 10, 20, 30, 40], 0.95), 38)

const sample = distribution([10, 20, 30])
assert.equal(sample.count, 3)
assert.equal(sample.minMs, 10)
assert.equal(sample.meanMs, 20)
assert.equal(sample.p50Ms, 20)
assert.equal(sample.p95Ms, 29)
assert.ok(Math.abs((sample.p99Ms ?? 0) - 29.8) < 1e-12)
assert.equal(sample.maxMs, 30)

console.log('latency statistics test passed')
