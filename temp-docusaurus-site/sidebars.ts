import type {SidebarsConfig} from '@docusaurus/plugin-content-docs';

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

/**
 * Creating a sidebar enables you to:
 - create an ordered group of docs
 - render a sidebar for each doc of that group
 - provide next/previous navigation

 The sidebars can be generated from the filesystem, or explicitly defined here.

 Create as many sidebars as you want.
 */
const sidebars: SidebarsConfig = {
  // By default, Docusaurus generates a sidebar from the docs folder structure
  tutorialSidebar: [
    'Introduction', // Ensure Introduction.md is listed first
    {
      type: 'category',
      label: 'Chapter 1', // Label for Chapter 1
      link: {
        type: 'docSidebar',
        sidebar: 'tutorialSidebar',
        slug: '/category/chapter-1' // Path to the category
      },
      items: [
        'chapter1/index',
        'chapter1/1.1-introduction',
        'chapter1/1.2-custom-interfaces',
        'chapter1/1.3-nodes-topics',
        'chapter1/1.4-services',
        'chapter1/1.5-actions',
        'chapter1/1.6-packages-launch-qos',
      ],
    },
  ],

  // But you can create a sidebar manually
  /*
  tutorialSidebar: [
    'intro',
    'hello',
    {
      type: 'category',
      label: 'Tutorial',
      items: ['tutorial-basics/create-a-document'],
    },
  ],
   */
};

export default sidebars;
