import React from 'react';
import PropTypes from 'prop-types';
import { Grid } from '@mui/material';
import Tabs from '@mui/material/Tabs';
import Tab from '@mui/material/Tab';
import Typography from '@mui/material/Typography';
import Box from '@mui/material/Box';
import styles from './index.module.css';
import Image from '../Image';

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
            developments. You can check out our contribution’s guidelines on GitHub and how to
            contribute on Luos Documentation.
          </p>
          <p>
            If you have any question about this roadmap or more generally about Luos, join in the
            community and chat with developers from all around the world!
          </p>
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
              Lorem Ipsum is simply dummy text of the printing and typesetting industry. Lorem Ipsum
              has been the industry's standard dummy text ever since the 1500s, when an unknown
              printer took a galley of type and scrambled it to make a type specimen book. It has
              survived not only five centuries, but also the leap into electronic typesetting,
              remaining essentially unchanged. It was popularised in the 1960s with the release of
              Letraset sheets containing Lorem Ipsum passages, and more recently with desktop
              publishing software like Aldus PageMaker including versions of Lorem Ipsum.
            </TabPanel>
            <TabPanel value={value} index={1}>
              Lorem Ipsum is simply dummy text of the printing and typesetting industry. Lorem Ipsum
              has been the industry's standard dummy text ever since the 1500s, when an unknown
              printer took a galley of type and scrambled it to make a type specimen book. It has
              survived not only five centuries, but also the leap into electronic typesetting,
              remaining essentially unchanged. It was popularised in the 1960s with the release of
              Letraset sheets containing Lorem Ipsum passages, and more recently with desktop
              publishing software like Aldus PageMaker including versions of Lorem Ipsum.
            </TabPanel>
          </Box>
        </Grid>
      </Grid>
    </div>
  );
};

export default Roadmap;
