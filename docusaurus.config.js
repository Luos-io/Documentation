const lightCodeTheme = require('prism-react-renderer/themes/github');
const darkCodeTheme = require('prism-react-renderer/themes/dracula');

/** @type {import('@docusaurus/types').DocusaurusConfig} */
module.exports = {
  title: 'Luos',
  tagline:
    'Open source and real-time orchestrator for distributed architectures',
  url: 'https://luos.io',
  baseUrl: '/',
  onBrokenLinks: 'throw',
  onBrokenMarkdownLinks: 'throw',
  favicon: 'img/favicon.png',
  organizationName: 'Luos-io', // Usually your GitHub org/user name.
  projectName: 'Documentation', // Usually your repo name.
  themeConfig: {
    navbar: {
      logo: {
        alt: 'Luos Logo',
        src: 'img/logo_luos_animated_black.gif',
        srcDark: 'img/logo_luos_animated_white.gif',
      },
      items: [
        {
          href: 'https://www.luos.io/#technology',
          label: 'Technology',
          position: 'right',
        },
        {
          href: 'https://www.luos.io/support',
          label: 'Pricing',
          position: 'right',
        },
        {
          href: 'https://www.luos.io/appstore',
          label: 'Appstore',
          position: 'right',
        },
        { to: '/blog', label: 'Blog', position: 'right' },
        {
          type: 'doc',
          label: 'Documentation',
          docId: 'get-started/getting-started',
          position: 'right',
        },
        { to: '/tutorials/tutorials', label: 'Tutorials', position: 'right' },
        { to: '/faq/list', label: 'FAQ', position: 'right' },

        {
          href: 'https://github.com/luos-io',
          label: 'Download',
          position: 'right',
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
              label: 'Documentation',
              to: '/docs/get-started/getting-started',
            },
          ],
        },
        {
          title: 'Community',
          items: [
            {
              label: 'Reddit',
              href: 'https://www.reddit.com/r/Luos/',
            },
            {
              label: 'Linkedin',
              href: 'https://www.linkedin.com/company/luos',
            },
            {
              label: 'Twitter',
              href: 'https://twitter.com/Luos_io',
            },
          ],
        },
        {
          title: 'More',
          items: [
            {
              label: 'GitHub',
              href: 'https://github.com/luos-io',
            },
            {
              label: 'Youtube',
              to: 'https://www.youtube.com/channel/UCWeIoHVY9Z-04kdwXNtv2FA',
            },
            {
              label: 'Blog',
              to: '/blog',
            },
          ],
        },
      ],
      copyright: `Copyright © ${new Date().getFullYear()} Luos, Inc. Built with Docusaurus.`,
    },
    prism: {
      theme: lightCodeTheme,
      darkTheme: darkCodeTheme,
    },
  },
  presets: [
    [
      '@docusaurus/preset-classic',
      {
        docs: {
          sidebarPath: require.resolve('./sidebarsDocs.js'),
          // Please change this to your repo.
          editUrl:
            'https://github.com/facebook/docusaurus/edit/master/website/',
        },
        blog: {
          showReadingTime: true,
          // Please change this to your repo.
          editUrl:
            'https://github.com/facebook/docusaurus/edit/master/website/blog/',
        },
        theme: {
          customCss: require.resolve('./src/css/custom.css'),
        },
      },
    ],
  ],
  plugins: [
    [
      '@docusaurus/plugin-content-docs',
      {
        id: 'tutorials',
        path: 'tutorials',
        routeBasePath: 'tutorials',
        sidebarPath: require.resolve('./sidebarsTutorials.js'),
        // ... autres options
      },
    ],
    [
      '@docusaurus/plugin-content-docs',
      {
        id: 'faq',
        path: 'faq',
        routeBasePath: 'faq',
        sidebarPath: require.resolve('./sidebarsFaq.js'),
        // ... autres options
      },
    ],
  ],
};
