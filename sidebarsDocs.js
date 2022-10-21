module.exports = {
  documentation: [
    {
      type: 'category',
      label: 'Luos technology',
      link: {
        type: 'doc',
        id: 'luos-technology/index',
      },
      items: [
        {
          type: 'category',
          label: 'Basics',
          link: {
            type: 'doc',
            id: 'luos-technology/basics/index',
          },
          items: [
            {
              type: 'doc',
              label: 'Concept',
              id: 'luos-technology/basics/concepts',
            },
            {
              type: 'doc',
              label: 'Code organization',
              id: 'luos-technology/basics/organization',
            },
          ],
        },
        {
          type: 'category',
          label: 'Node',
          link: {
            type: 'doc',
            id: 'luos-technology/node/node',
          },
          items: [
            {
              type: 'doc',
              label: 'Luos engine',
              id: 'luos-technology/node/luos',
            },
            {
              type: 'doc',
              label: 'Hardware abstraction layer',
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
          label: 'Packages',
          id: 'luos-technology/packages/index',
        },
        {
          type: 'category',
          label: 'Services',
          link: {
            type: 'doc',
            id: 'luos-technology/services/index',
          },
          items: [
            {
              type: 'doc',
              label: 'Create services',
              id: 'luos-technology/services/create-services',
            },
            {
              type: 'doc',
              label: 'Types',
              id: 'luos-technology/services/types',
            },
            {
              type: 'doc',
              label: 'Profiles',
              id: 'luos-technology/services/profiles',
            },
            {
              type: 'doc',
              label: 'Routing table',
              id: 'luos-technology/services/routing-table',
            },
            {
              type: 'doc',
              label: 'Timestamp',
              id: 'luos-technology/services/timestamp',
            },
          ],
        },
        {
          type: 'category',
          label: 'Messages',
          link: {
            type: 'doc',
            id: 'luos-technology/messages/index',
          },
          items: [
            {
              type: 'doc',
              label: 'Send message',
              id: 'luos-technology/messages/basic-message',
            },
            {
              type: 'doc',
              label: 'Receive message',
              id: 'luos-technology/messages/handling-message',
            },
            {
              type: 'doc',
              label: 'Commands',
              id: 'luos-technology/messages/command',
            },
            {
              type: 'doc',
              label: 'Object dictionnary',
              id: 'luos-technology/messages/object-dictionary',
            },
            {
              type: 'doc',
              label: 'Advanced message',
              id: 'luos-technology/messages/advanced-message',
            },
          ],
        },
      ],
    },
    {
      type: 'category',
      label: 'Hardware consideration',
      link: {
        type: 'doc',
        id: 'hardware-consideration/hardware-consideration',
      },
      items: [
        {
          type: 'doc',
          label: 'Minimum requirements',
          id: 'hardware-consideration/minimum-requirements',
        },
        {
          type: 'doc',
          label: 'Luos engine configuration',
          id: 'hardware-consideration/mcu',
        },
        {
          type: 'doc',
          label: 'Electronic design',
          id: 'hardware-consideration/electronics',
        },
        {
          type: 'doc',
          label: 'Test your Robus configuration',
          id: 'hardware-consideration/test-your-configuration',
        },
      ],
    },
    {
      type: 'category',
      label: 'Built-in tools',
      link: {
        type: 'doc',
        id: 'tools/index',
      },
      items: [
        {
          type: 'doc',
          label: 'Gate',
          id: 'tools/gate',
        },
        {
          type: 'doc',
          label: 'JSON API',
          id: 'tools/api-json',
        },
        {
          type: 'doc',
          label: 'Luos engine api',
          id: 'tools/luos-engine-api',
        },
        {
          type: 'doc',
          label: 'Bootloader',
          id: 'tools/bootloader',
        },
        {
          type: 'doc',
          label: 'Inspector',
          id: 'tools/inspector',
        },
        {
          type: 'doc',
          label: 'Monitoring',
          id: 'tools/monitoring',
        },
        {
          type: 'doc',
          label: 'PlatformIO',
          id: 'tools/platformio',
        },
        {
          type: 'link',
          label: 'Network display',
          href: 'https://app.luos.io/network',
        },
      ],
    },
    {
      type: 'category',
      label: 'Software Integrations',
      link: {
        type: 'doc',
        id: 'integrations/index',
      },
      items: [
        {
          type: 'doc',
          label: 'SDK python: Pyluos',
          id: 'integrations/pyluos',
        },
        {
          type: 'link',
          label: 'SDK typescript',
          href: 'https://github.com/Luos-io/sdk-ts',
        },
        {
          type: 'link',
          label: 'PlatformIO',
          href: 'https://www.luos.io/tutorials/pio',
        },
        {
          type: 'link',
          label: 'Arduino IDE',
          href: 'https://www.luos.io/tutorials/arduino',
        },
        {
          type: 'link',
          label: 'Espressif IDE',
          href: 'https://www.luos.io/tutorials/esp',
        },
        {
          type: 'link',
          label: 'RTOS',
          href: 'https://www.luos.io/tutorials/freertos',
        },
        {
          type: 'doc',
          label: 'ROS',
          id: 'integrations/ros',
        },
        {
          type: 'doc',
          label: 'SimpleFOC',
          id: 'integrations/simplefoc',
        },
      ],
    },
    {
      type: 'category',
      label: 'Compatibility',
      items: [
        {
          type: 'doc',
          label: 'Luos ecosystem',
          id: 'compatibility/ecosystem',
        },
        {
          type: 'doc',
          label: 'Boards compatibility',
          id: 'compatibility/mcu_demoboard',
        },
      ],
    },
    {
      type: 'category',
      label: 'Advanced documentation',
      link: {
        type: 'doc',
        id: 'advanced-documentation/index',
      },
      items: [
        {
          type: 'category',
          label: 'Robus',
          link: {
            type: 'doc',
            id: 'advanced-documentation/robus/index',
          },
          items: [
            {
              type: 'doc',
              label: 'The transmission, and reception management',
              id: 'advanced-documentation/robus/transmission',
            },
            {
              type: 'doc',
              label: 'The topology management',
              id: 'advanced-documentation/robus/topology',
            },
            {
              type: 'doc',
              label: 'The protocol definition',
              id: 'advanced-documentation/robus/protocol',
            },
            {
              type: 'doc',
              label: 'The physical driver interface',
              id: 'advanced-documentation/robus/physical-driver-interface',
            },
          ],
        },
      ],
    },
    {
      type: 'doc',
      label: '🖊️ Contribute',
      id: 'contribute-to-luos',
    },
  ],
};
