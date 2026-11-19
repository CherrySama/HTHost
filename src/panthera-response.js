// Read once: a missing reply does not mean a control request failed to execute.
export async function readPantheraResponse(response) {
  let data;
  try {
    data = await response.json();
  } catch {
    // A proxy may return HTML or an empty body instead of the bridge response.
  }
  if (data === null || typeof data !== 'object' || Array.isArray(data)) {
    throw new Error(`真机服务返回无效响应（HTTP ${response.status}），请求结果未确认`);
  }
  return data;
}
