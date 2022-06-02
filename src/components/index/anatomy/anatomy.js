import React, { useState } from 'react';
import styles from './anatomy.module.css';
import Grid from '@mui/material/Grid';
import List from '@mui/material/List';
import ListItem from '@mui/material/ListItem';
import ListItemText from '@mui/material/ListItemText';
import ListItemAvatar from '@mui/material/ListItemAvatar';
import Link from '@mui/material/Link';
import Radio from '@mui/material/Radio';
import RadioGroup from '@mui/material/RadioGroup';
import FormControlLabel from '@mui/material/FormControlLabel';
import KeyboardArrowDownIcon from '@mui/icons-material/KeyboardArrowDown';
import { useColorMode } from '@docusaurus/theme-common';
import FileCopyIcon from '@mui/icons-material/FileCopy';
import SearchIcon from '@mui/icons-material/Search';
import MemoryIcon from '@mui/icons-material/Memory';
import MiscellaneousServicesIcon from '@mui/icons-material/MiscellaneousServices';
import ComputerIcon from '@mui/icons-material/Computer';
import AccountTreeIcon from '@mui/icons-material/AccountTree';
import CalendarMonthIcon from '@mui/icons-material/CalendarMonth';

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
      <Grid container>
        <Grid item xs={12} md={6} lg={6} xl={6}>
          {' '}
          <h2 className={`${styles.title} ${styles.underline}`}>
            Anatomy of Luos
          </h2>
          <p className={styles.text}>
            Luos gives you access to multiple features created by developers for
            developers. Many of these features are based on the needs of the
            community.
          </p>
        </Grid>
      </Grid>

      <Grid container mt={5} mb={5}>
        <Grid item md={3} lg={3} xl={4.5}>
          <List sx={{ width: '100%' }} className={styles.list}>
            <ListItem
              alignitems="flex-start"
              component={Link}
              href="/docs/luos-technology/basics#introduction-to-luos"
              className={styles.listLink}
            >
              <ListItemAvatar>
                <ComputerIcon fontSize="large" className={styles.listIcon} />
              </ListItemAvatar>
              <ListItemText
                primary="Remote control"
                secondary={
                  'You can access the topology and routing table anywhere, even on your computer, another machine, or a cloud application.'
                }
              />
            </ListItem>

            <ListItem
              alignitems="flex-start"
              component={Link}
              href="/tutorials/bootloader/intro"
              className={styles.listLink}
            >
              <ListItemAvatar>
                <AccountTreeIcon fontSize="large" className={styles.listIcon} />
              </ListItemAvatar>
              <ListItemText
                style={{ color: 'black !important' }}
                primary="Bootloader"
                secondary={
                  'Luos engine allows you to update any firmware of your device, from anywhere.'
                }
              />
            </ListItem>
            <ListItem
              alignitems="flex-start"
              component={Link}
              href="/tutorials/your-first-detection/"
              className={styles.listLink}
            >
              <ListItemAvatar>
                <MemoryIcon fontSize="large" className={styles.listIcon} />
              </ListItemAvatar>
              <ListItemText
                primary="Topology and routing table"
                secondary={
                  'Luos detects all the services in your system and locates them. It allows you to access and adapt to any feature from anywhere.'
                }
              />
            </ListItem>
          </List>
        </Grid>
        <Grid item md={3} lg={3} xl={4.5}>
          <List
            sx={{ width: '100%' }}
            display="flex"
            className={`${styles.list} ${styles.secondList}`}
          >
            <ListItem
              alignitems="flex-start"
              component={Link}
              href="/tutorials/get-started/get-started2"
              className={styles.listLink}
            >
              <ListItemAvatar>
                <SearchIcon fontSize="large" className={styles.listIcon} />
              </ListItemAvatar>
              <ListItemText
                primary="Monitoring"
                secondary={
                  'You can control and monitor your device with several SDKs (Python, TS, Browser app, and others - coming soon).'
                }
              />
            </ListItem>

            <ListItem
              alignitems="flex-start"
              component={Link}
              href="/docs/luos-technology/services/timestamp"
              className={styles.listLink}
            >
              <ListItemAvatar>
                <CalendarMonthIcon
                  fontSize="large"
                  className={styles.listIcon}
                />
              </ListItemAvatar>
              <ListItemText
                primary="Timestamp"
                secondary={
                  'Luos engine provides you with a distributed timestamp management system.'
                }
              />
            </ListItem>
            <ListItem
              alignitems="flex-start"
              component={Link}
              href="/docs/luos-technology/basics#introduction-to-luos"
              className={styles.listLink}
            >
              <ListItemAvatar>
                <MiscellaneousServicesIcon
                  fontSize="large"
                  className={styles.listIcon}
                />
              </ListItemAvatar>
              <ListItemText
                primary="Microservice architecture"
                secondary={
                  'Luos follows the microservices philosophy. Luos engine is a library that allows you to conceptualize your features using small, independent and loosely coupled bricks.'
                }
              />
            </ListItem>
          </List>
        </Grid>
        <Grid item xs={12} md={6} lg={6} xl={3} className={styles.vscode}>
          <div className={styles.head}>
            {' '}
            <img
              src="img/index/header/buttons.svg"
              style={{ padding: '15px', width: '100px' }}
            />
          </div>
          <Grid container style={{ height: '87%' }}>
            <Grid item xs={2} md={2} lg={1} className={styles.docs}>
              <FileCopyIcon fontSize="large" className={styles.icons} />
              <SearchIcon fontSize="large" className={styles.icons} />
              {isDarkTheme ? (
                <>
                  {' '}
                  <img src="img/index/header/luos.svg" />{' '}
                </>
              ) : (
                <>
                  {' '}
                  <img src="img/index/header/luos-white.svg" />{' '}
                </>
              )}
            </Grid>
            <Grid item xs={4} md={4} lg={3} xl={3} className={styles.video}>
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
              xl={8}
              className={styles.player}
              style={{
                backgroundImage: `url('/img/index/anatomy/illu/${currentImageTmp}.svg')`,
              }}
            ></Grid>
          </Grid>
          <Grid item xs={12} className={styles.footer}>
            {' '}
          </Grid>
        </Grid>
      </Grid>
      <Grid container>
        <Grid item xs={4} md={4} lg={4} xl={4} className={styles.lines}>
          <img
            src="img/index/integration/line-right.svg"
            style={{ transform: 'rotateY(180deg)' }}
            className={styles.linesImg}
          />
        </Grid>
        <Grid item xs={4} md={4} lg={4} xl={4}></Grid>
        <Grid item xs={4} md={4} lg={4} xl={4} className={styles.lines}>
          <img
            src="img/index/integration/line-right.svg"
            style={{ float: 'right' }}
            className={styles.linesImg}
          />
        </Grid>
      </Grid>
    </div>
  );
};
export default Anatomy;
