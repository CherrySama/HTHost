// Python 桥接独立于网页存活；这里只管理网页生命周期通知，不发送运动命令。
export function createBridgePageLifecycle({ enabled, storage = sessionStorage, events = window, request = fetch }) {
  const bridgePageKey = "panthera.bridgePage";
  let bridgePage;
  try {
    bridgePage = JSON.parse(storage.getItem(bridgePageKey) || "null");
  } catch { /* Storage may be unavailable. */ }
  bridgePage ||= { page: crypto.randomUUID(), session: null };
  let pageLifecycleStamp = bridgePage.stamp || 0;
  function notifyBridgePage(leaving) {
    if (!bridgePage.session || !enabled()) return;
    pageLifecycleStamp = Math.max(Date.now(), pageLifecycleStamp + 1);
    bridgePage.stamp = pageLifecycleStamp;
    try { storage.setItem(bridgePageKey, JSON.stringify(bridgePage)); } catch {}
    void request("/api/panthera/connection/page", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ ...bridgePage, stamp: pageLifecycleStamp, leaving }),
      keepalive: true,
    }).catch(() => {});
  }
  events.addEventListener("pagehide", () => notifyBridgePage(true));
  events.addEventListener("pageshow", () => notifyBridgePage(false));

  return {
    setSession(session) {
      if (!session || bridgePage.session === session) return;
      bridgePage.session = session;
      try { storage.setItem(bridgePageKey, JSON.stringify(bridgePage)); } catch {}
      notifyBridgePage(false);
    },
  };
}
