import React from 'react';
import { BrowserView } from 'react-device-detect';
import styles from './header.module.css';
import Grid from '@mui/material/Grid';
import Button from '@mui/material/Button';
import Radio from '@mui/material/Radio';
import RadioGroup from '@mui/material/RadioGroup';
import FormControlLabel from '@mui/material/FormControlLabel';
import Writer from '@site/src/components/writer';
import RocketLaunchIcon from '@mui/icons-material/RocketLaunch';
import KeyboardArrowDownIcon from '@mui/icons-material/KeyboardArrowDown';
import { useColorMode } from '@docusaurus/theme-common';
import FileCopyIcon from '@mui/icons-material/FileCopy';
import SearchIcon from '@mui/icons-material/Search';
import PlayArrowIcon from '@mui/icons-material/PlayArrow';

import LiteYouTubeEmbed from 'react-lite-youtube-embed';
import 'react-lite-youtube-embed/dist/LiteYouTubeEmbed.css';

const videos = [
  {
    label: 'What is Luos',
    id: 'ujh0xNE3TZ8',
  },
  {
    label: 'Bike alarm',
    id: 'kTr-pwIrRqw',
  },
  {
    label: 'Demonstration',
    id: 'KHHB_jfBDEY',
  },
];

const Header = () => {
  const { isDarkTheme } = useColorMode();
  const [currentVideo, setCurrentVideo] = React.useState(videos[0]);

  const handleChange = (event) => {
    console.log('TEST', event.target.value);
    setCurrentVideo(videos.find(({ id }) => event.target.value === id));
  };

  return (
    <div className={styles.container}>
      <Grid container>
        <Grid item md={7} className={styles.left}>
          <div className={styles.head}>
            {' '}
            <img
              src="assets/images/index/header/buttons.svg"
              alt="buttons"
              style={{ padding: '15px', width: '100px' }}
            />
          </div>
          <Grid container>
            <Grid item xs={2} md={1} lg={1} className={styles.docs}>
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

            <Grid item xs={10} md={11} lg={11} pl={3} className={styles.playerLeft}>
              <p className={styles.text}>
                The devtool dedicated to cyber-physical systems making hardware as modular as
                software.
              </p>
              <Writer
                textStyle={{
                  color: '#BD99FF',
                  height: '70px',
                  marginTop: '15px',
                }}
                startDelay={0}
                cursorColor="#BD99FF"
                multiTextLoop={true}
                multiText={[
                  'an embedded microservice orchestrator_',
                  'a helpful developers community_',
                  'an easy-to-integrate library_',
                ]}
                multiTextDelay={2000}
                typeSpeed={100}
              />{' '}
              <p className={styles.text}>
                Don't waste time to create complex software architectures anymore. Luos allows you
                to easily develop and scale your <b>cyber-physical systems</b>. Our engine is free
                and open source.
              </p>
              <div className={styles.btnContainer}>
                <Button
                  variant="contained"
                  className={styles.pinkBtn}
                  href="/tutorials/get-started"
                >
                  Get started <RocketLaunchIcon style={{ marginLeft: '10px' }} />
                </Button>
                <Button
                  variant="contained"
                  className={styles.whiteBtn}
                  href="https://discord.gg/luos"
                  rel="nofollow"
                >
                  Join the community
                </Button>
                <Button
                  variant="contained"
                  className={styles.videoBtn}
                  href="https://www.youtube.com/watch?v=ujh0xNE3TZ8"
                  rel="nofollow"
                >
                  Watch our video <PlayArrowIcon />
                </Button>
              </div>
            </Grid>
          </Grid>
          <Grid item xs={12} md={12} lg={12} xl={12} className={styles.footer}>
            {' '}
          </Grid>
        </Grid>

        <Grid item md={5} lg={5}></Grid>
      </Grid>
      <BrowserView>
        <Grid container className={styles.mobileNone}>
          <Grid item md={6} lg={6}></Grid>
          <Grid
            item
            md={6}
            mt={-13}
            style={{
              zIndex: '2',
              paddingLeft: '0 !important',
            }}
          >
            <div className={styles.head}>
              {' '}
              <img
                src="assets/images/index/header/buttons.svg"
                alt="buttons"
                style={{ padding: '15px', width: '100px' }}
              />
            </div>
            <Grid container>
              <Grid item md={1} lg={1} className={styles.docs}>
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
              <Grid item md={2.5} className={styles.video}>
                <RadioGroup
                  className={styles.videoList}
                  value={currentVideo.id}
                  onChange={handleChange}
                >
                  {videos.map(({ id, label }, i) => (
                    <FormControlLabel
                      key={`header-video-${i}`}
                      value={id}
                      className={
                        currentVideo.id === id ? styles.videoListItemActive : styles.videoListItem
                      }
                      control={<Radio />}
                      label={label}
                      sx={{ margin: '0 !important', padding: '8px 0', fontSize: '0.9rem' }}
                    />
                  ))}
                </RadioGroup>
              </Grid>
              <Grid item md={8.5} className={styles.player}>
                <LiteYouTubeEmbed
                  className={styles.player_iframe}
                  id={currentVideo.id}
                  videotitle={currentVideo.label}
                ></LiteYouTubeEmbed>
              </Grid>
            </Grid>
            <Grid item md={12} lg={12} xl={12} className={styles.footer}>
              {' '}
            </Grid>
          </Grid>
        </Grid>
      </BrowserView>
    </div>
  );
};
export default Header;
