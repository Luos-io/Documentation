module.exports = {
  documentation: [
    {
      type: 'category',
      label: 'Tutorials',
      link: { type: 'doc', id: 'index' },
      items: [
        {
          type: 'category',
          label: 'Get Started',
          link: { type: 'doc', id: 'get-started/get-started' },
          items: [
            {
              type: 'doc',
              label: 'Part 1: Your first Luos service',
              id: 'get-started/get-started1',
            },
            {
              type: 'doc',
              label: 'Part 2: Take the control',
              id: 'get-started/get-started2',
            },
            {
              type: 'doc',
              label: 'Part 3: Unleash your code',
              id: 'get-started/get-started3',
            },
            {
              type: 'doc',
              label: 'Part 4: Connect to a web app',
              id: 'get-started/get-started4',
            },
          ],
        },
        {
          type: 'category',
          label: 'Your first service',
          link: { type: 'doc', id: 'your-first-service/your-first-service' },
          items: [
            {
              type: 'doc',
              label: 'Luos service',
              id: 'your-first-service/luos-service',
            },
            {
              type: 'doc',
              label: 'Create a package',
              id: 'your-first-service/create-a-package',
            },
          ],
        },
        {
          type: 'category',
          label: 'Your first message',
          link: { type: 'doc', id: 'your-first-message/your-first-message' },
          items: [
            {
              type: 'doc',
              label: 'Receiving a message',
              id: 'your-first-message/receiving-message',
            },
            {
              type: 'doc',
              label: 'Send a message from button service',
              id: 'your-first-message/send-message',
            },
          ],
        },
        {
          type: 'category',
          label: 'Your first topology detection',
          link: {
            type: 'doc',
            id: 'your-first-detection/your-first-detection',
          },
          items: [
            {
              type: 'doc',
              label: 'The topology of your system',
              id: 'your-first-detection/topology',
            },
            {
              type: 'doc',
              label: 'How to use the routing table',
              id: 'your-first-detection/routing-table',
            },
            {
              type: 'doc',
              label: 'Full embedded application',
              id: 'your-first-detection/embedded-app',
            },
          ],
        },
        {
          type: 'category',
          label: 'Arduino',
          link: { type: 'doc', id: 'arduino/intro' },
          items: [
            {
              type: 'doc',
              label: 'Use Luos with Arduino IDE',
              id: 'arduino/arduino',
            },
          ],
        },
        {
          type: 'category',
          label: 'RTOS',
          link: { type: 'doc', id: 'freertos/intro' },
          items: [
            {
              type: 'doc',
              label: 'Use Luos with an RTOS',
              id: 'freertos/freertos',
            },
          ],
        },
        {
          type: 'category',
          label: 'Bootloader',
          link: { type: 'doc', id: 'bootloader/intro' },
          items: [
            {
              type: 'doc',
              label: 'Use Luos bootloader',
              id: 'bootloader/bootloader',
            },
          ],
        },
        {
          type: 'category',
          label: 'PlatformIo',
          link: { type: 'doc', id: 'pio/pio' },
          items: [
            {
              type: 'doc',
              label: 'Project creation',
              id: 'pio/creation',
            },
            {
              type: 'doc',
              label: 'Include Luos to your project',
              id: 'pio/include',
            },
          ],
        },
        {
          type: 'category',
          label: 'Alias',
          link: { type: 'doc', id: 'resilient-alias/intro' },
          items: [
            {
              type: 'doc',
              label: 'Use resilient aliases',
              id: 'resilient-alias/resilient-alias',
            },
          ],
        },
        {
          type: 'category',
          label: 'Morse encoder',
          link: { type: 'doc', id: 'morse/morse' },
          items: [
            {
              type: 'doc',
              label: 'Algorithm',
              id: 'morse/algorithm',
            },
            {
              type: 'doc',
              label: 'Output',
              id: 'morse/output',
            },
            {
              type: 'doc',
              label: 'Add service',
              id: 'morse/add-service',
            },
          ],
        },
        {
          type: 'category',
          label: 'Luos integration',
          link: {
            type: 'doc',
            id: 'luos-integration/list',
          },
          items: [
            {
              type: 'doc',
              label: 'Install ROS 2',
              id: 'luos-integration/install-ros2',
            },
            {
              type: 'doc',
              label: 'ROS 2 package example',
              id: 'luos-integration/ros2-package-example',
            },
            {
              type: 'doc',
              label: 'ROS 1 retro-compatibility with Luos',
              id: 'luos-integration/ros1-retrocompatibility',
            },
          ],
        },
      ],
    },
  ],
};
