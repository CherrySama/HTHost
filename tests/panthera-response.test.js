import test from 'node:test';
import assert from 'node:assert/strict';
import { readPantheraResponse } from '../src/panthera-response.js';

test('rejects malformed, empty and non-object bridge responses with HTTP context', async () => {
  for (const body of ['', '<html>Bad Gateway</html>', 'null', '[]', '1']) {
    await assert.rejects(readPantheraResponse(new Response(body, { status: 502 })), /HTTP 502.*结果未确认/);
  }
});

test('preserves structured bridge status and errors for the caller', async () => {
  for (const [status, body] of [[200, {state: 'connected'}], [409, {message: '请先停止 SDK 动作'}]]) {
    assert.deepEqual(await readPantheraResponse(new Response(JSON.stringify(body), {status})), body);
  }
});
