// @ts-nocheck
// Docusaurus plugin to inject the ChatWidget globally
// This plugin injects the chat widget into all pages using the Root component

const path = require('path');

module.exports = function (context, options) {
  return {
    name: 'docusaurus-chatbot-plugin',

    getClientModules() {
      return [path.resolve(__dirname, './ChatWidgetInjector')];
    },
  };
};