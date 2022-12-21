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
import SvgIcon from '@mui/material/SvgIcon';
import KeyboardArrowDownIcon from '@mui/icons-material/KeyboardArrowDown';
import { useColorMode } from '@docusaurus/theme-common';
import AccountTreeIcon from '@mui/icons-material/AccountTree';
import ComputerIcon from '@mui/icons-material/Computer';
import FileCopyIcon from '@mui/icons-material/FileCopy';
import LinkIcon from '@mui/icons-material/Link';
import SearchIcon from '@mui/icons-material/Search';
import ViewTimelineIcon from '@mui/icons-material/ViewTimeline';

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

  const DiscordIcon = (props) => (
    <SvgIcon {...props}>
      <g id="b">
        <path
          class="h"
          d="M26.72,0H5.28C3.47,0,2,1.47,2,3.3V24.93c0,1.82,1.47,3.3,3.28,3.3H23.42l-.85-2.96,2.05,1.9,1.94,1.79,3.44,3.04V3.3c0-1.82-1.47-3.3-3.28-3.3Z"
        />
        <path
          style={{
            fill: '#fff',
          }}
          d="M19.39,19.6c.48,.61,1.06,1.3,1.06,1.3,3.54-.11,4.9-2.43,4.9-2.43,0-5.15-2.3-9.33-2.3-9.33-2.3-1.73-4.5-1.68-4.5-1.68l-.22,.26c2.72,.83,3.98,2.03,3.98,2.03-1.66-.91-3.3-1.36-4.82-1.54-1.15-.13-2.26-.1-3.23,.03-.1,0-.18,.02-.27,.03-.56,.05-1.92,.26-3.63,1.01-.59,.27-.94,.46-.94,.46,0,0,1.33-1.26,4.21-2.1l-.16-.19s-2.19-.05-4.5,1.68c0,0-2.3,4.18-2.3,9.33,0,0,1.34,2.32,4.88,2.43,0,0,.59-.72,1.07-1.33-2.03-.61-2.8-1.89-2.8-1.89,0,0,.16,.11,.45,.27,.02,.02,.03,.03,.06,.05,.05,.03,.1,.05,.14,.08,.4,.22,.8,.4,1.17,.54,.66,.26,1.44,.51,2.35,.69,1.2,.22,2.61,.3,4.14,.02,.75-.13,1.52-.35,2.32-.69,.56-.21,1.18-.51,1.84-.94,0,0-.8,1.31-2.9,1.9Zm-6.38-2.62c-.9,0-1.63-.8-1.63-1.78s.72-1.78,1.63-1.78,1.65,.8,1.63,1.78c0,.98-.72,1.78-1.63,1.78Zm5.84,0c-.9,0-1.63-.8-1.63-1.78s.72-1.78,1.63-1.78,1.63,.8,1.63,1.78-.72,1.78-1.63,1.78Z"
        />
      </g>
    </SvgIcon>
  );

  return (
    <div className={styles.container}>
      <Grid container>
        <Grid item xs={12} md={6} lg={6} xl={6}>
          {' '}
          <h2 className={`${styles.title} ${styles.underline}`}>Benefits of Luos</h2>
          <p className={styles.text}>
            Luos gives you access to multiple features created by developers for developers. Many of
            these features are based on the needs of the community.
          </p>
        </Grid>
      </Grid>

      <Grid container mt={5} mb={5}>
        <Grid item md={3} lg={3} xl={4.5}>
          <List sx={{ width: '100%' }} className={styles.list}>
            <ListItem
              alignitems="flex-start"
              component={Link}
              href="/docs/integrations/pyluos"
              className={styles.listLink}
            >
              <ListItemAvatar>
                <ComputerIcon fontSize="large" className={styles.listIcon} />
              </ListItemAvatar>
              <ListItemText
                primary="Digital Twins"
                secondary={
                  'You can natively remote-control any service anywhere on your computer, on another machine, or in a cloud application.'
                }
              />
            </ListItem>
            <ListItem
              alignitems="flex-start"
              component={Link}
              href="/docs/integrations"
              className={styles.listLink}
            >
              <ListItemAvatar>
                <LinkIcon fontSize="large" className={styles.listIcon} />
              </ListItemAvatar>
              <ListItemText
                style={{ color: 'black !important' }}
                primary="Integrations"
                secondary={'Luos provides existing bridges with multiple other technologies.'}
              />
            </ListItem>
            <ListItem
              alignitems="flex-start"
              component={Link}
              href="/tutorials/bootloader"
              className={styles.listLink}
            >
              <ListItemAvatar>
                <AccountTreeIcon fontSize="large" className={styles.listIcon} />
              </ListItemAvatar>
              <ListItemText
                style={{ color: 'black !important' }}
                primary="Deployment"
                secondary={
                  'The bootloader feature allows you to update any firmware of your cyber-physical systems from anywhere.'
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
              href="https://discord.gg/luos"
              rel="external nofollow"
              className={styles.listLink}
            >
              <ListItemAvatar>
                <DiscordIcon fontSize="large" viewBox="0 0 32 32" className={styles.listIcon} />
              </ListItemAvatar>
              <ListItemText
                primary="Community"
                secondary={
                  'Join our community to exchange with other developers and collaborate on projects.'
                }
              />
            </ListItem>
            <ListItem
              alignitems="flex-start"
              component={Link}
              href="/roadmap"
              className={styles.listLink}
            >
              <ListItemAvatar>
                <ViewTimelineIcon fontSize="large" className={styles.listIcon} />
              </ListItemAvatar>
              <ListItemText
                primary="Soon"
                secondary={"We are always attentive to developers' needs to give them superpowers."}
              />
            </ListItem>
          </List>
        </Grid>
        <Grid item xs={12} md={6} lg={6} xl={3} className={styles.vscode}>
          <div className={styles.head}>
            {' '}
            <img
              src="assets/images/index/header/buttons.svg"
              alt="buttons"
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
                  <img src="assets/images/index/header/luos.svg" alt="luos" />{' '}
                </>
              ) : (
                <>
                  {' '}
                  <img src="assets/images/index/header/luos-white.svg" alt="luos-white" />{' '}
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
                  className={currentInt == 'topo' ? styles.engineActive : styles.engine}
                  control={<Radio />}
                  label="Topology"
                />
                <FormControlLabel
                  value="micro"
                  className={currentInt == 'micro' ? styles.engineActive : styles.engine}
                  control={<Radio />}
                  label="Microservices"
                />
                <FormControlLabel
                  value="remote"
                  className={currentInt == 'remote' ? styles.engineActive : styles.engine}
                  control={<Radio />}
                  label="Remote control"
                />
                <FormControlLabel
                  value="bootloader"
                  className={currentInt == 'bootloader' ? styles.engineActive : styles.engine}
                  control={<Radio />}
                  label="Bootloader"
                />
                <FormControlLabel
                  value="timestamp"
                  className={currentInt == 'timestamp' ? styles.engineActive : styles.engine}
                  control={<Radio />}
                  label="Timestamp"
                />
                <FormControlLabel
                  value="monitoring"
                  className={currentInt == 'monitoring' ? styles.engineActive : styles.engine}
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
                backgroundImage: `url('assets/images/index/anatomy/illu/${currentImageTmp}.svg')`,
              }}
            ></Grid>
          </Grid>
          <Grid item xs={12} className={styles.footer}>
            {' '}
          </Grid>
        </Grid>
      </Grid>
      <Grid container sx={{ marginBottom: '-50px' }}>
        <Grid item xs={4} className={styles.lines}>
          <img
            src="assets/images/index/integration/line-right.svg"
            alt="line-right"
            style={{ transform: 'rotateY(180deg)' }}
            className={styles.linesImg}
          />
        </Grid>
        <Grid item xs={true}></Grid>
        <Grid item xs={4} className={styles.lines}>
          <img
            src="assets/images/index/integration/line-right.svg"
            alt="line-right"
            style={{ float: 'right' }}
            className={styles.linesImg}
          />
        </Grid>
      </Grid>
    </div>
  );
};
export default Anatomy;
