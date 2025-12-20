/**
 * Copyright (c) Facebook, Inc. and its affiliates.
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 */

// Mock an empty localStorage as it is not available in Node.js
const localStorage = {
  getItem: () => null,
  setItem: () => {},
  removeItem: () => {},
};

global.localStorage = localStorage;