import {themes as prismThemes} from 'prism-react-renderer';
import type {Config} from '@docusaurus/types';
import type * as Preset from '@docusaurus/preset-classic';

const config: Config = {
  title: 'PHYSICAL AI & HUMANOID ROBOTICS COURSE',
  tagline: 'Your comprehensive guide to building intelligent robotic systems.',
  favicon: 'img/favicon.jpg',

  url: 'https://muhammad-rehan.github.io',
  baseUrl: '/Physical-AI-Humanoid-Robotics-Textbook/',

  organizationName: 'Muhammad-Rehan',
  projectName: 'Physical-AI-Humanoid-Robotics-Textbook',

  onBrokenLinks: 'ignore',
  onBrokenMarkdownLinks: 'warn',

  i18n: {
    defaultLocale: 'en',
    locales: ['en'],
  },

  plugins: [
    '@docusaurus/plugin-ideal-image',
  ],

  presets: [
    [
      'classic',
      {
        docs: {
          sidebarPath: require.resolve('./sidebars.ts'),
          editUrl: 'https://github.com/Muhammad-Rehan/Physical-AI-Humanoid-Robotics-Textbook/edit/main/',
        },
        blog: {
          showReadingTime: true,
          feedOptions: {
            type: ['rss', 'atom'],
            xslt: true,
          },
          editUrl: 'https://github.com/Muhammad-Rehan/Physical-AI-Humanoid-Robotics-Textbook/edit/main/blog/',
          onInlineTags: 'warn',
          onInlineAuthors: 'warn',
          onUntruncatedBlogPosts: 'warn',
        },
        theme: {
          customCss: require.resolve('./src/css/custom.css'),
        },
      } satisfies Preset.Options,
    ],
  ],

  themeConfig: {
    image: 'img/docusaurus-social-card.jpg',
    colorMode: {
      respectPrefersColorScheme: true,
    },
    navbar: {
      title: 'Physical AI & Humanoid Robotics',
      logo: {
        alt: 'My Site Logo',
        src: 'img/logo.png',
      },
      items: [
        {
          type: 'doc',
          docId: 'intro/introduction',
          position: 'left',
          label: 'Book',
        },
      ],
    },
    footer: {
      style: 'dark',
      links: [
        {
          title: 'Docs',
          items: [
            {
              label: 'Textbook',
              to: '/docs/intro/introduction',
            },
          ],
        },
        {
          title: 'Community',
          items: [
            {label: 'Stack Overflow', href: '/'},
            {label: 'Discord', href: '/'},
            {label: 'X', href: '/'},
          ],
        },
        {
          title: 'More',
          items: [
            {label: 'Blog', to: '/blog'},
            {label: 'GitHub', href: 'https://github.com/Muhammad-Rehan/Physical-AI-Humanoid-Robotics-Textbook'},
          ],
        },
      ],
      copyright: `Copyright © ${new Date().getFullYear()} AI Robotics Textbook. Made By Muhammad Rehan.`,
    },
    prism: {
      theme: prismThemes.github,
      darkTheme: prismThemes.dracula,
    },
  } satisfies Preset.ThemeConfig,
};

export default config;
