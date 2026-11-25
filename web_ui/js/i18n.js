// i18n (spec 13.4): flat keys, data-i18n="key", missing keys show the key itself, language in localStorage.
const LANGS = ["en", "de", "it"];
const NUMBER = /^-?\d+\.\d+$/;
let texts = {};
let lang = "en";

/** Fill {name} placeholders; numbers get a decimal comma in German and Italian. Pure, for tests. */
export function render(template, args = {}, language = "en") {
  let out = template;
  for (const [name, raw] of Object.entries(args)) {
    let value = String(raw);
    if (language !== "en" && NUMBER.test(value)) value = value.replace(".", ",");
    out = out.split(`{${name}}`).join(value);
  }
  return out;
}

/** "name=value" list (ActivityState args) -> object. */
export function argsFromList(items = []) {
  const out = {};
  for (const item of items) {
    const i = item.indexOf("=");
    if (i > 0) out[item.slice(0, i)] = item.slice(i + 1);
  }
  return out;
}

export function t(key, args = {}) {
  return render(Object.prototype.hasOwnProperty.call(texts, key) ? texts[key] : key, args, lang);
}

export function currentLang() {
  return lang;
}

function storedLang() {
  try {
    const v = window.localStorage.getItem("robotros.lang");
    return LANGS.includes(v) ? v : null;
  } catch {
    return null;
  }
}

export async function setLang(newLang) {
  if (!LANGS.includes(newLang)) newLang = "en";
  try {
    const res = await fetch(`i18n/${newLang}.json`, { cache: "no-cache" });
    texts = res.ok ? await res.json() : {};
  } catch {
    texts = {};
  }
  lang = newLang;
  try {
    window.localStorage.setItem("robotros.lang", newLang);
  } catch {
    /* storage may be unavailable (spec 13.4: try/catch) */
  }
  document.documentElement.lang = newLang;
  applyDom();
  window.dispatchEvent(new CustomEvent("robotros:lang", { detail: newLang }));
}

export function applyDom(root = document) {
  for (const el of root.querySelectorAll("[data-i18n]")) el.textContent = t(el.dataset.i18n);
}

export function initLang() {
  return setLang(storedLang() || "en");
}

export { LANGS };
