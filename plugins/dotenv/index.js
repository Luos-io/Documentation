const Dotenv = require('dotenv-webpack');

/** @type {Dotenv.Options} */
const DEFAULT_OPTIONS = {
  path: './.env', // The path to your environment variables.
  safe: false, // If false ignore safe-mode, if true load './.env.example', if a string load that file as the sample
  systemvars: false, // Set to true if you would rather load all system variables as well (useful for CI purposes)
  silent: false, //  If true, all warnings will be suppressed
  expand: false, // Allows your variables to be "expanded" for reusability within your .env file
  defaults: false, //  Adds support for dotenv-defaults. If set to true, uses ./.env.defaults
};

/** @type {import('@docusaurus/types').Plugin} */
module.exports = (_context, opts) => ({
  name: 'dotenv',
  configureWebpack() {
    return {
      plugins: [new Dotenv({ ...DEFAULT_OPTIONS, ...opts })],
      module: {
        rules: [],
      },
    };
  },
});
