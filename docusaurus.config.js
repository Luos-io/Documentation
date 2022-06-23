const lightCodeTheme = require('prism-react-renderer/themes/github');
const darkCodeTheme = require('prism-react-renderer/themes/dracula');

/** @type {import('@docusaurus/types').DocusaurusConfig} */
module.exports = {
  title: 'Luos',
  tagline:
    'Luos makes it easy to develop and scale your edge and embedded distributed software. It is open source.',
  url: 'https://www.luos.io',
  baseUrl: '/',
  onBrokenLinks: 'throw',
  onBrokenMarkdownLinks: 'throw',
  favicon: 'img/favicon.png',
  organizationName: 'Luos-io', // Usually your GitHub org/user name.
  projectName: 'Documentation', // Usually your repo name.

  customFields: {
    node_def:
      'Hardware element (MCU) hosting and running Luos and hosting one or several services.',
    service_def:
      'Software element run by Luos that can communicate with other services. It can be a driver or an app.',
    od_def:
      'Set of objects based on SI metric system that can be transmitted through Luos messages. Any object can easily be converted in other units.',
    microservices_def:
      'Microservices are a software development technique that arranges an application as a collection of loosely coupled services.',
    robus_def: 'Bus communication protocol used by Luos.',
    luoshal_def: 'Hardware Abstraction Layer used to fit Luos with various hardware designs.',
    last_version_pyluos: '2.0.0',
    last_version_luos: ' 2.0.1',
    gh_path: 'github.com/Luos-io/doc/tree/master/src',
  },

  themeConfig: {
    metadata: [
      {
        name: 'description',
        content:
          'Luos makes it easy to develop and scale your edge and embedded distributed software. It is open source.',
      },
    ],
    image: 'img/thumbnail-luos.png',
    colorMode: {
      defaultMode: 'light',
    },
    hotjar: {
      applicationId: '2480203',
    },
    docs: {
      sidebar: {
        hideable: true,
      },
    },
    navbar: {
      logo: {
        alt: 'Luos Logo',
        src: 'img/logo_luos_animated_black.gif',
        srcDark: 'img/logo_luos_animated_white.gif',
      },
      items: [
        {
          type: 'docsVersionDropdown',
          position: 'left',
          dropdownActiveClassDisabled: true,
        },
        {
          to: '/',
          label: 'Technology',
          position: 'right',
        },
        {
          to: 'https://app.luos.io',
          label: 'Tools',
          position: 'right',
        },
        {
          type: 'dropdown',
          label: 'Resources',
          position: 'right',
          items: [
            {
              to: '/tutorials/get-started',
              label: 'Get started',
            },
            {
              to: '/tutorials',
              label: 'Tutorials',
            },
            {
              to: '/docs/luos-technology',
              label: 'Documentation',
            },
            {
              to: '/faq',
              label: 'Troubleshooting',
            },
            {
              to: '/blog',
              label: 'Blog',
            },
          ],
        },
        {
          type: 'dropdown',
          label: 'Community',
          position: 'right',
          items: [
            {
              to: 'https://discord.gg/luos',
              label: 'Discord',
            },
            {
              to: 'https://www.reddit.com/r/Luos/',
              label: 'Reddit',
            },
            {
              to: 'https://github.com/luos-io',
              label: 'Github',
            },
          ],
        },
        {
          href: 'https://github.com/luos-io',
          className: 'header-github-link',
          'aria-label': 'GitHub repository',
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
              to: '/docs/luos-technology',
            },
            {
              label: 'Contact us',
              to: '/feedbacks/send',
            },
          ],
        },
        {
          title: 'Community',
          items: [
            {
              label: 'Discord',
              href: 'https://discord.gg/luos',
            },
            {
              label: 'Reddit',
              href: 'https://www.reddit.com/r/Luos/',
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
              href: 'https://www.youtube.com/channel/UCWeIoHVY9Z-04kdwXNtv2FA',
            },
          ],
        },
      ],
      copyright: `Copyright © ${new Date().getFullYear()} Luos, built with Docusaurus.`,
    },

    prism: {
      theme: lightCodeTheme,
      darkTheme: darkCodeTheme,
    },

    algolia: {
      appId: 'K3VMDT0LOA',
      apiKey: '9394b39227bc70e30ff8a34bc6489a3f',
      indexName: 'docs-luos',
      contextualSearch: true,
    },
  },

  presets: [
    [
      '@docusaurus/preset-classic',
      {
        docs: {
          sidebarPath: require.resolve('./sidebarsDocs.js'),
          // Please change this to your repo.
          editUrl: 'https://github.com/luos-io/Documentation',
          versions: {
            current: {
              label: '2.5.0-beta 🚧',
            },
          },
        },

        blog: {
          //sidebarPath: require.resolve('./sidebarsBlog.js'),
          blogTitle: 'Luos Blog',
          blogDescription: 'A blog about microservices and CI/CD in Edge and Embedded systems',
          postsPerPage: 'ALL',
          blogSidebarTitle: 'All posts',
          blogSidebarCount: 'ALL',
          showReadingTime: true, // When set to false, the "x min read" won't be shown
          readingTime: ({ content, frontMatter, defaultReadingTime }) =>
            defaultReadingTime({ content, options: { wordsPerMinute: 300 } }),
        },

        theme: { customCss: require.resolve('./src/css/custom.css') },
        gtag: { trackingID: 'GTM-M73ZRR4' },
        googleAnalytics: { trackingID: 'UA-153509818-3' },
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
      },
    ],
    [
      '@docusaurus/plugin-content-docs',
      {
        id: 'faq',
        path: 'faq',
        routeBasePath: 'faq',
      },
    ],
    [
      '@docusaurus/plugin-content-pages',
      {
        id: 'feedbacks',
        path: 'feedbacks',
        routeBasePath: 'feedbacks',
      },
    ],
    [
      '@docusaurus/plugin-client-redirects',
      {
        fromExtensions: ['html'],
      },
    ],
    ['./plugins/dotenv', {}],
    ['./plugins/hotjar', {}],
  ],
};
