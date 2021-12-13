module.exports = {
  documentation: [
    {
      type: 'category',
      label: 'Documentation',
      items: [
        {
          type: 'category',
          label: 'Luos Technology',
          items: [
            {
              type: 'doc',
              label: 'Definition',
              id: 'luos-technology/luos_tech',
            },
            {
              type: 'category',
              label: 'Basics',
              items: [
                {
                  type: 'doc',
                  label: 'Basics',
                  id: 'luos-technology/basics/basics',
                },
                {
                  type: 'doc',
                  label: 'Concept',
                  id: 'luos-technology/basics/concept',
                },
                {
                  type: 'doc',
                  label: 'Code organization',
                  id: 'luos-technology/basics/orga',
                },
              ],
            },
            {
              type: 'category',
              label: 'Node',
              items: [
                {
                  type: 'doc',
                  label: 'Definition',
                  id: 'luos-technology/node/node',
                },
                {
                  type: 'doc',
                  label: 'Luos',
                  id: 'luos-technology/node/luos',
                },
                {
                  type: 'doc',
                  label: 'Luos HAL',
                  id: 'luos-technology/node/luos-hal',
                },
                {
                  type: 'doc',
                  label: 'Network topology',
                  id: 'luos-technology/node/topology',
                },
              ],
            },
            {
              type: 'doc',
              label: 'Package',
              id: 'luos-technology/package/package',
            },
            {
              type: 'category',
              label: 'Services',
              items: [
                {
                  type: 'doc',
                  label: 'Definition',
                  id: 'luos-technology/services/services',
                },
                {
                  type: 'doc',
                  label: 'Initialization',
                  id: 'luos-technology/services/service-api',
                },
                {
                  type: 'doc',
                  label: 'Types',
                  id: 'luos-technology/services/service-type',
                },
                {
                  type: 'doc',
                  label: 'Profiles',
                  id: 'luos-technology/services/profile',
                },
                {
                  type: 'doc',
                  label: 'Routing Table',
                  id: 'luos-technology/services/routing-table',
                },
              ],
            },
            {
              type: 'category',
              label: 'Message',
              items: [
                {
                  type: 'doc',
                  label: 'Definition',
                  id: 'luos-technology/message/message',
                },
                {
                  type: 'doc',
                  label: 'Send Message',
                  id: 'luos-technology/message/basic-message',
                },
                {
                  type: 'doc',
                  label: 'Receive Message',
                  id: 'luos-technology/message/handling-message',
                },
                {
                  type: 'doc',
                  label: 'Commands',
                  id: 'luos-technology/message/command',
                },
                {
                  type: 'doc',
                  label: 'Object dictionnary',
                  id: 'luos-technology/message/object-dictionary',
                },
                {
                  type: 'doc',
                  label: 'Advanced message',
                  id: 'luos-technology/message/advanced-message',
                },
              ],
            },
          ],
        },
        {
          type: 'category',
          label: 'Hardware Consideration',
          items: [
            {
              type: 'doc',
              label: 'Definition',
              id: 'hardware-consideration/hardware-consideration',
            },
            {
              type: 'doc',
              label: 'Minimum Requirement',
              id: 'hardware-consideration/minimum-requirement',
            },
            {
              type: 'doc',
              label: 'Luos configuration',
              id: 'hardware-consideration/mcu',
            },
            {
              type: 'doc',
              label: 'Electronic Design',
              id: 'hardware-consideration/electronics',
            },
            {
              type: 'doc',
              label: 'Test Your Configuration',
              id: 'hardware-consideration/test-your-configuration',
            },
          ],
        },
        {
          type: 'category',
          label: 'Tools',
          items: [
            {
              type: 'doc',
              label: 'List',
              id: 'tools/tool',
            },
            {
              type: 'doc',
              label: 'Gate',
              id: 'tools/gate',
            },
            {
              type: 'doc',
              label: 'Pyluos',
              id: 'tools/pyluos',
            },
            {
              type: 'doc',
              label: 'Bootloader',
              id: 'tools/boot',
            },
            {
              type: 'doc',
              label: 'Monitoring',
              id: 'tools/monitoring',
            },
            {
              type: 'doc',
              label: 'ROS',
              id: 'tools/ros',
            },
          ],
        },
        {
          type: 'category',
          label: 'API',
          items: [
            {
              type: 'doc',
              label: 'List',
              id: 'api/list',
            },
            {
              type: 'doc',
              label: 'JSON API',
              id: 'api/api-json',
            },
          ],
        },
      ],
    },
    { type: 'link', label: 'Go to Luos.io', href: 'https://www.luos.io/' },
  ],
};
