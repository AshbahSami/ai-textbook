// setup-localStorage-polyfill.js
// This file provides a simple polyfill for localStorage for Node.js environment
// during Docusaurus build process.

if (typeof global.localStorage === 'undefined') {
  global.localStorage = {
    _data: {},
    setItem: function (id, val) {
      return (this._data[id] = String(val));
    },
    getItem: function (id) {
      return this._data.hasOwnProperty(id) ? this._data[id] : undefined;
    },
    removeItem: function (id) {
      return delete this._data[id];
    },
    clear: function () {
      return (this._data = {});
    },
  };
}
