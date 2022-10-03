import React from 'react';
import PropTypes from 'prop-types';
import { Button, Grid } from '@mui/material';
import Tabs from '@mui/material/Tabs';
import Tab from '@mui/material/Tab';
import Typography from '@mui/material/Typography';
import Box from '@mui/material/Box';
import styles from './index.module.css';
import Image from '../Image';
import Link from '@docusaurus/Link';

function TabPanel(props) {
  const { children, value, index, ...other } = props;

  return (
    <div
      role="tabpanel"
      hidden={value !== index}
      id={`simple-tabpanel-${index}`}
      aria-labelledby={`simple-tab-${index}`}
      {...other}
    >
      {value === index && (
        <Box sx={{ p: 3 }}>
          <Typography className={styles.tab}>{children}</Typography>
        </Box>
      )}
    </div>
  );
}

TabPanel.propTypes = {
  children: PropTypes.node,
  index: PropTypes.number.isRequired,
  value: PropTypes.number.isRequired,
};

function a11yProps(index) {
  return {
    id: `simple-tab-${index}`,
    'aria-controls': `simple-tabpanel-${index}`,
  };
}

const Roadmap = () => {
  const [value, setValue] = React.useState(0);

  const handleChange = (event, newValue) => {
    setValue(newValue);
  };

  return (
    <div>
      <h1 className={styles.title}>Luos Roadmap</h1>
      <Grid container justifyContent="center">
        <Grid item md={8} mb={5}>
          <p>
            This page is a preview of Luos’s product roadmap. This graphic shows the next technical
            developments our team is and will be working on with the help of the community, on a
            one-year scale.
          </p>
          <p>This roadmap is updated every quarter.</p>
          <p>
            As part of an open source project, each of these subjects are open to contribution: it
            means that anyone from the Luos community can work on them and help the team in their
            developments. You can check out our{' '}
            <Link
              href="https://github.com/Luos-io/luos_engine/blob/main/CONTRIBUTING.md"
              target="_blank"
            >
              contribution’s guidelines on GitHub
            </Link>{' '}
            and{' '}
            <Link href="https://www.luos.io/docs/contribute-to-luos" target="_blank">
              how to contribute on Luos Documentation
            </Link>
            .
          </p>
          <p>
            If you have any question about this roadmap or more generally about Luos, join in the
            community and chat with developers from all around the world!
          </p>

          <Box textAlign="center">
            <Button href="https://discord.gg/luos" className={styles.btn}>
              Join the community
            </Button>
          </Box>
        </Grid>
        <Grid item md={8} mb={5}>
          <Image
            src="assets/images/roadmap/public-roadmap-light.png"
            darkSrc="assets/images/roadmap/public-roadmap-dark.png"
          />
        </Grid>
        <Grid item md={8} mb={5}>
          <Box sx={{ width: '100%' }}>
            <Box sx={{ borderBottom: 1, borderColor: 'divider' }}>
              <Tabs
                value={value}
                onChange={handleChange}
                TabIndicatorProps={{ style: { background: '#bd99ff' } }}
              >
                <Tab label="Luos Engine" {...a11yProps(0)} className={styles.tabLabel} />
                <Tab label="Luos Tools" {...a11yProps(1)} className={styles.tabLabel} />
              </Tabs>
            </Box>
            <TabPanel value={value} index={0}>
              <h2 className={styles.subtitle}>Luos engine v3</h2>
              <h5>Q1 2023</h5>
              <p>
                The main goal of Luos engine v3 is to drastically simplify the addition of various
                networks’ support into the Luos engine library. Luos engine will not be limited to
                the Robus network layer anymore, and will be able to deal with any kind of network.
              </p>

              <h2 className={styles.subtitle}>Multi-phy</h2>
              <h5>Q3 2023</h5>
              <p>
                Based on Luos engine v3, the multi-phy feature will allow Luos engine to deal with
                multiple and different networks at the same time (WiFi, CAN, Robus, …), acting like
                a switch. This feature will allow you to deal with multiple networks on your device,
                multiply data bandwidth, and break the frontier between your embedded networks and
                your cloud or computer applications.
              </p>
              <h2 className={styles.subtitle}>Luos web assembly</h2>
              <h5>Q1 2024</h5>
              <p>
                Thanks to multi-phy, you can now run Luos engine services everywhere, even on the
                cloud, on your computer, or on a mobile phone. To make it even more flexible, we
                will compile Luos engine in WebAssembly, allowing you to run services directly in
                your web browser. This web browser application will be visible by any embedded
                services and usable from anywhere.
              </p>
              <h2 className={styles.subtitle}>Luos certification</h2>
              <h5>Q3 2024</h5>
              <p>
                Luos engine certification will add connectivity rules to your nodes, allowing you to
                create white-lists, black-lists, or any conditional rules based on encrypted
                information on all your nodes. Your product, your rules.
              </p>
            </TabPanel>
            <TabPanel value={value} index={1}>
              <h2 className={styles.subtitle}>Luos hub</h2>
              <h5>Q3 2023</h5>
              <p>
                Luos hub is a registry allowing you to deal with your Luos packages revision as you
                want, and to easily reuse and share the services with other users.
              </p>

              <h2 className={styles.subtitle}>Luos inspector</h2>
              <h5>Q1 2024</h5>
              <p>
                Luos inspector will be a web application allowing you to inspect any event happening
                on your whole product at a nanosecond scale. You can use it to deeply understand and
                debug your entire product.
              </p>
              <h2 className={styles.subtitle}>Luos manager</h2>
              <h5>Q3 2024</h5>
              <p>
                Luos manager will provide a complete CI/CD interface allowing you to manage your
                product during the development phases and on post-production. This tool will allow
                you to easily manage your products’ configuration from A to Z, inspect any product,
                link bugs with inspection cession, or deploy a specific configuration to specific
                targets.
              </p>
            </TabPanel>
          </Box>
          <Box textAlign="center">
            <Button href="https://discord.gg/luos" className={styles.btn}>
              Join the community
            </Button>
          </Box>
        </Grid>
      </Grid>
    </div>
  );
};

export default Roadmap;
