// temp-docusaurus-site/jest.config.js
module.exports = {
  preset: 'ts-jest',
  testEnvironment: 'jsdom',
  testMatch: ['<rootDir>/src/**/*.test.ts', '<rootDir>/src/**/*.test.tsx'],
  moduleNameMapper: {
    '^@theme/(.*)$': '<rootDir>/src/theme/$1',
    '^@docusaurus/(.*)$': '<rootDir>/node_modules/@docusaurus/$1',
    '\.(css|less|scss|sass)$': 'identity-obj-proxy',
  },
  globals: {
    'ts-jest': {
      tsconfig: 'tsconfig.json',
    },
  },
};
