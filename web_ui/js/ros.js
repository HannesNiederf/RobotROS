// rosbridge connection (spec 13.1/13.2): ws://<host of the page>:9090, reconnect with 1 -> 5 s backoff,
// subscriptions are restored after a reconnect, onConnect hooks run after every (re)connect
// (e.g. /map/get_snapshot). ROSLIB is the global from vendor/roslib.min.js.
import * as state from "./state.js";

const subs = [];
const connectHooks = [];
let ros = null;
let backoff = 1;
let everConnected = false;

export function rosbridgeUrl(loc = window.location) {
  const params = new URLSearchParams(loc.search);
  return params.get("rosbridge") || `ws://${loc.hostname || "localhost"}:9090`;
}

function attach(sub) {
  sub.topic = new ROSLIB.Topic({
    ros,
    name: sub.name,
    messageType: sub.type,
    throttle_rate: sub.throttleMs || 0,
    queue_length: 1,
  });
  sub.topic.subscribe(sub.cb);
}

export function connect(url = rosbridgeUrl()) {
  state.set("connection", everConnected ? "lost" : "connecting");
  ros = new ROSLIB.Ros({ url });
  ros.on("connection", () => {
    everConnected = true;
    backoff = 1;
    state.set("connection", "connected");
    subs.forEach(attach);
    connectHooks.forEach((fn) => fn());
  });
  ros.on("error", () => {});
  ros.on("close", () => {
    state.set("connection", everConnected ? "lost" : "connecting");
    const wait = backoff;
    backoff = Math.min(5, backoff + 1);
    setTimeout(() => connect(url), wait * 1000);
  });
}

export function subscribe(name, type, cb, throttleMs = 0) {
  const sub = { name, type, cb, throttleMs, topic: null };
  subs.push(sub);
  if (ros && ros.isConnected) attach(sub);
}

export function onConnect(fn) {
  connectHooks.push(fn);
}

export function publish(name, type, msg) {
  if (!ros || !ros.isConnected) return false;
  new ROSLIB.Topic({ ros, name, messageType: type }).publish(msg);
  return true;
}

export function callService(name, type, request = {}) {
  return new Promise((resolve, reject) => {
    if (!ros || !ros.isConnected) {
      reject(new Error("not connected"));
      return;
    }
    new ROSLIB.Service({ ros, name, serviceType: type }).callService(request, resolve, reject);
  });
}
