import React, { useState } from 'react';
import styles from './anatomy.module.css';
import Grid from '@mui/material/Grid';
import List from '@mui/material/List';
import ListItem from '@mui/material/ListItem';
import ListItemText from '@mui/material/ListItemText';
import ListItemAvatar from '@mui/material/ListItemAvatar';
import Avatar from '@mui/material/Avatar';
import Radio from '@mui/material/Radio';
import RadioGroup from '@mui/material/RadioGroup';
import FormControlLabel from '@mui/material/FormControlLabel';
import KeyboardArrowDownIcon from '@mui/icons-material/KeyboardArrowDown';
import { useColorMode } from '@docusaurus/theme-common';

const Anatomy = (props) => {
  const { isDarkTheme } = useColorMode();
  const img = {
    topo: 'topology',
    micro: 'microservice',
    remote: 'remote',
    bootloader: 'bootloader',
    timestamp: 'timestamp',
    monitoring: 'monitoring',
  };
  const [currentImageTmp, setCurrentImageTmp] = useState(img['topo']);
  const [currentInt, setCurrentInt] = React.useState('topo');

  const handleChange = (event) => {
    setCurrentInt(event.target.value);
    setCurrentImageTmp(img[event.target.value]);
  };
  return (
    <div className={styles.container}>
      <h1 className={`${styles.title} ${styles.underline}`}>Anatomy of Luos</h1>
      <p className={styles.text}>
        Luos gives you access to multiple features developed by developers for
        developers. Some of the features come from community needs.
      </p>
      <Grid container mt={5} mb={5}>
        <Grid item md={4} lg={4} xl={4.5}>
          <List sx={{ width: '100%' }} className={styles.list}>
            <ListItem alignItems="flex-start">
              <ListItemAvatar>
                <Avatar
                  alt="table tree icon"
                  src="/img/index/anatomy/table-tree.svg"
                  variant="square"
                />
              </ListItemAvatar>
              <ListItemText
                primary="Topology and routing table"
                secondary={
                  'Luos detects every services in your system and locates them. Allowing you to access and adapt to any feature from anywhere.'
                }
              />
            </ListItem>

            <ListItem alignItems="flex-start">
              <ListItemAvatar>
                <Avatar
                  alt="cubes icon"
                  src="/img/index/anatomy/cubes.svg"
                  variant="square"
                />
              </ListItemAvatar>
              <ListItemText
                primary="Microservice architecture"
                secondary={
                  'Luos follows the microservice philosophy. Luos Engine is a library allowing you to think your features into small independant and loosly coupled bricks.'
                }
              />
            </ListItem>

            <ListItem alignItems="flex-start">
              <ListItemAvatar>
                <Avatar
                  alt="files icon"
                  src="/img/index/anatomy/files.svg"
                  variant="square"
                />
              </ListItemAvatar>
              <ListItemText
                primary="Remote control"
                secondary={
                  'You can access the topology and the routing table anywhere, even your computer, another machine or a cloud application.'
                }
              />
            </ListItem>
          </List>
        </Grid>
        <Grid item md={4} lg={4} xl={4.5}>
          <List sx={{ width: '100%' }} display="flex" className={styles.list}>
            <ListItem alignItems="flex-start">
              <ListItemAvatar>
                <Avatar
                  alt="bootloader icon"
                  src="/img/index/anatomy/bootloader.svg"
                  variant="square"
                />
              </ListItemAvatar>
              <ListItemText
                style={{ color: 'black !important' }}
                primary="Bootloader"
                secondary={
                  'Luos Engine allows you to update any firmware of your device, from anywhere.'
                }
              />
            </ListItem>

            <ListItem alignItems="flex-start">
              <ListItemAvatar>
                <Avatar
                  alt="timestamp icon"
                  src="/img/index/anatomy/timestamp.svg"
                  variant="square"
                />
              </ListItemAvatar>
              <ListItemText
                primary="Timestamp"
                secondary={
                  'Luos Engine provides you a distributed timestamp management system.'
                }
              />
            </ListItem>

            <ListItem alignItems="flex-start">
              <ListItemAvatar>
                <Avatar
                  alt="glass icon"
                  src="/img/index/anatomy/glass.svg"
                  variant="square"
                />
              </ListItemAvatar>
              <ListItemText
                primary="Monitoring"
                secondary={
                  'You can control and monitor your device with several SDK (Python, TS, Browser app, and others soon)'
                }
              />
            </ListItem>
          </List>
        </Grid>
        <Grid item xs={12} md={4} lg={4} xl={3} className={styles.vscode}>
          <div className={styles.head}>
            {' '}
            <img
              src="img/index/header/buttons.svg"
              style={{ padding: '15px' }}
            />
          </div>
          <Grid container style={{ height: '87%' }}>
            <Grid item xs={2} md={2} lg={1} className={styles.docs}>
              {isDarkTheme ? (
                <>
                  {' '}
                  <img src="img/index/header/docs.svg" />
                  <img src="img/index/header/zoom.svg" />
                  <img src="img/index/header/luos.svg" />{' '}
                </>
              ) : (
                <>
                  {' '}
                  <img src="img/index/header/docs-white.svg" />
                  <img src="img/index/header/zoom-white.svg" />
                  <img src="img/index/header/luos-white.svg" />{' '}
                </>
              )}
            </Grid>
            <Grid item xs={4} md={4} lg={3} className={styles.video}>
              <span>
                <KeyboardArrowDownIcon className={styles.cardIcons} /> Anatomy
              </span>

              <RadioGroup
                aria-labelledby="demo-controlled-radio-buttons-group"
                name="controlled-radio-buttons-group"
                value={currentInt}
                onChange={handleChange}
              >
                <FormControlLabel
                  value="topo"
                  className={
                    currentInt == 'topo' ? styles.engineActive : styles.engine
                  }
                  control={<Radio />}
                  label="Topology"
                />
                <FormControlLabel
                  value="micro"
                  className={
                    currentInt == 'micro' ? styles.engineActive : styles.engine
                  }
                  control={<Radio />}
                  label="Microservices"
                />
                <FormControlLabel
                  value="remote"
                  className={
                    currentInt == 'remote' ? styles.engineActive : styles.engine
                  }
                  control={<Radio />}
                  label="Remote control"
                />
                <FormControlLabel
                  value="bootloader"
                  className={
                    currentInt == 'bootloader'
                      ? styles.engineActive
                      : styles.engine
                  }
                  control={<Radio />}
                  label="Bootloader"
                />
                <FormControlLabel
                  value="timestamp"
                  className={
                    currentInt == 'timestamp'
                      ? styles.engineActive
                      : styles.engine
                  }
                  control={<Radio />}
                  label="Timestamp"
                />
                <FormControlLabel
                  value="monitoring"
                  className={
                    currentInt == 'monitoring'
                      ? styles.engineActive
                      : styles.engine
                  }
                  control={<Radio />}
                  label="Monitoring"
                />
              </RadioGroup>
            </Grid>
            <Grid
              item
              xs={6}
              md={6}
              lg={8}
              className={styles.player}
              style={{
                backgroundImage: `url('/img/index/anatomy/illu/${currentImageTmp}.svg')`,
              }}
            ></Grid>
          </Grid>
        </Grid>
      </Grid>
      <Grid container>
        <Grid item md={4} lg={4} xl={4} className={styles.lines}>
          <img src="/img/index/anatomy/line-left.svg" />
        </Grid>
        <Grid item md={4} lg={4} xl={4}></Grid>
        <Grid item md={4} lg={4} xl={4} className={styles.lines}>
          <img
            src="/img/index/anatomy/line-right.svg"
            style={{ float: 'right' }}
          />
        </Grid>
      </Grid>
    </div>
  );
};
export default Anatomy;
