module.exports = {
  documentation: [
    {
      type: 'category',
      label: 'Documentation',
      items: [
        {
          type: 'doc',
          label: 'Getting Started',
          id: 'get-started/getting-started',
        },
        {
          type: 'category',
          label: 'Luos Technology',
          items: [
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
              label: 'Service',
              items: [
                {
                  type: 'doc',
                  label: 'Definition',
                  id: 'luos-technology/services/services',
                },
                {
                  type: 'doc',
                  label: 'Initilization',
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
              label: 'MCU with Luos',
              id: 'hardware-consideration/mcu',
            },
            {
              type: 'doc',
              label: 'Electronic Design',
              id: 'hardware-consideration/electronics',
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
        {
          type: 'category',
          label: 'Tutorials',
          items: [
            {
              type: 'category',
              label: 'Luos and tools',
              items: [
                {
                  type: 'doc',
                  label: 'List',
                  id: 'tutorials/luos-and-tools/list',
                },
                {
                  type: 'doc',
                  label: 'Bootloader',
                  id: 'tutorials/luos-and-tools/bootloader',
                },
              ],
            },
            {
              type: 'category',
              label: 'Demo boards',
              items: [
                {
                  type: 'doc',
                  label: 'List',
                  id: 'tutorials/demo-boards/list',
                },
                {
                  type: 'doc',
                  label: 'Luos demo boards',
                  id: 'tutorials/demo-boards/luos-demo-boards',
                },
                {
                  type: 'doc',
                  label: 'Basic button-LED device',
                  id: 'tutorials/demo-boards/button-led',
                },
                {
                  type: 'doc',
                  label: 'Basic servomotor-LED device (video)',
                  id: 'tutorials/demo-boards/servomotor',
                },
                {
                  type: 'doc',
                  label: 'Use Luos with Arduino IDE',
                  id: 'tutorials/demo-boards/arduino',
                },
              ]
            },
            {
              type: 'category',
              label: 'Luos integration',
              items: [
                {
                  type: 'doc',
                  label: 'List',
                  id : 'tutorials/luos-integration/list',
                },
                {
                  type: 'doc',
                  label: 'Install ROS 2',
                  id : 'tutorials/luos-integration/install-ros2',
                },
                {
                  type: 'doc',
                  label: 'ROS 2 package example',
                  id : 'tutorials/luos-integration/ros2-package-example',
                },
                {
                  type: 'doc',
                  label: 'ROS 1 retro-compatibility with Luos',
                  id : 'tutorials/luos-integration/ros1-retrocompatibility',
                },
              ]
            }
          ],
        },
      ],
    },
  ],
};
