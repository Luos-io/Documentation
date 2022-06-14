import React, { useState } from 'react';
import styles from './integration.module.css';
import Grid from '@mui/material/Grid';
import Link from '@docusaurus/Link';
import Button from '@mui/material/Button';
import Radio from '@mui/material/Radio';
import RadioGroup from '@mui/material/RadioGroup';
import FormControlLabel from '@mui/material/FormControlLabel';
import KeyboardArrowDownIcon from '@mui/icons-material/KeyboardArrowDown';
import { useColorMode } from '@docusaurus/theme-common';
import ImageList from '@mui/material/ImageList';
import ImageListItem from '@mui/material/ImageListItem';
import ImageListItemBar from '@mui/material/ImageListItemBar';
import FileCopyIcon from '@mui/icons-material/FileCopy';
import SearchIcon from '@mui/icons-material/Search';

// DO NOT REMOVE WIP INTEGRATION BLOCK

const Integration = (props) => {
  const { isDarkTheme } = useColorMode();

  const img = {
    mcu: [
      ['esp', 'ESP32'],
      ['stm', 'STM32'],
      ['microship', 'Microchip'],
      ['raspberry', 'Raspberry Pi'],
      ['arduino', 'Arduino'],
      ['pio', 'Platform IO'],
      ['eclipse', 'Eclipse'],
      ['vscodeico', 'VS Code'],
    ],
    os: [
      ['ros', 'ROS-1 & ROS-2', 'ros-white'],
      ['freertos-white', 'FreeRTOS', 'freertos-white'],
      ['microros', 'Micro-ROS', 'microros-white'],
    ],
    api: [
      ['simplefoc', 'SimpleFOC'],
      ['zappier', 'Zappier '],
      ['ifttt', 'IFTTT ', 'ifttt-white'],
      ['freedom', 'Freedom Robotics '],
    ],
    sdk: [
      ['c', 'C/C++'],
      ['python', 'Python'],
      ['js', 'Javascript'],
      ['ts', 'Typescript'],
    ],
  };

  const soon = ['zappier', 'ifttt', 'freedom', 'microros'];

  const links = {
    esp: '',
    stm: '/docs/compatibility/mcu_demoboard#st',
    microship: '/docs/compatibility/mcu_demoboard',
    raspberry: '',
    arduino: '/tutorials/arduino/intro',
    pio: '/tutorials/get-started/get-started1#2-set-up-your-development-environment',
    eclipse: '/docs/luos-technology/basics/orga#luos-engines-levels',
    vscodeico: '/docs/luos-technology/basics/orga#luos-engines-levels',
    ros: '/docs/tools/ros',
    freertos: '/docs/tools/ros',
    microros: '',
    simplefoc: '',
    zappier: '',
    ifttt: '',
    freedom: '',
    c: '',
    python: '/docs/tools/pyluos#required-installing-python-and-pip',
    js: '',
    ts: '',
  };

  const [currentImageTmp, setCurrentImageTmp] = useState(img['mcu']);
  const [currentInt, setCurrentInt] = React.useState('mcu');

  const handleChange = (event) => {
    setCurrentInt(event.target.value);
    setCurrentImageTmp(img[event.target.value]);
  };

  let index = 0;

  return (
    <div className={styles.container}>
      <Grid container spacing={3}>
        <Grid item xs={6} md={4} lg={4}>
          {' '}
          <img src="img/index/integration/line-right.svg" className={styles.imgLeft} />
        </Grid>
        <Grid item xs={12} md={4} lg={4}>
          <h2 className={`${styles.title} ${styles.underline}`}>Integrations</h2>
        </Grid>
      </Grid>
      <Grid container spacing={3} sx={{ padding: '30px' }} className={styles.mobileNone}>
        <Grid item xs={1} md={1} lg={1} xl={1}></Grid>
        <Grid
          item
          md={10}
          lg={10}
          xl={10}
          style={{
            minHeight: '400px',
          }}
          className={styles.vscode}
        >
          <div className={styles.head}>
            <Grid container>
              <Grid item md={4} lg={4} xl={4}>
                {' '}
                <img
                  src="img/index/header/buttons.svg"
                  style={{ padding: '15px', width: '100px' }}
                />
              </Grid>
              <Grid item md={4} lg={4} xl={4} style={{ textAlign: 'center' }}>
                {' '}
                <h2 className={`${styles.hiwTitle} ${styles.underline}`}>Integrations</h2>
              </Grid>
              <Grid item md={4} lg={4} xl={4}></Grid>
            </Grid>
          </div>
          <Grid container style={{ height: '87%' }}>
            <Grid item md={1} lg={1} className={styles.docs}>
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
            <Grid item md={2.5} lg={2} xl={2} className={styles.video}>
              <span>
                <KeyboardArrowDownIcon className={styles.cardIcons} /> Integrations
              </span>

              <RadioGroup
                aria-labelledby="demo-controlled-radio-buttons-group"
                name="controlled-radio-buttons-group"
                value={currentInt}
                onChange={handleChange}
              >
                <FormControlLabel
                  value="mcu"
                  className={currentInt == 'mcu' ? styles.engineActive : styles.engine}
                  control={<Radio />}
                  label="MCU & IDE"
                />
                <FormControlLabel
                  value="os"
                  className={currentInt == 'os' ? styles.engineActive : styles.engine}
                  control={<Radio />}
                  label="OS & frameworks"
                />
                <FormControlLabel
                  value="api"
                  className={currentInt == 'api' ? styles.engineActive : styles.engine}
                  control={<Radio />}
                  label="APIs & 
                  integrations"
                />
                <FormControlLabel
                  value="sdk"
                  className={currentInt == 'sdk' ? styles.engineActive : styles.engine}
                  control={<Radio />}
                  label="SDK & 
                  languages"
                />
              </RadioGroup>
            </Grid>
            <Grid item md={8.5} lg={9} xl={9} className={styles.player}>
              <ImageList cols={4} style={{ padding: '15px' }}>
                {currentImageTmp.map((element, index) => (
                  <a
                    href={links[element[0]]}
                    className={links[element[0]] !== '' ? styles.imgLink : styles.imgLinkDesible}
                  >
                    <ImageListItem key={index} alignitems="center">
                      <img
                        src={
                          isDarkTheme
                            ? element[2]
                              ? `img/index/integration/icons/${element[2]}.svg`
                              : `img/index/integration/icons/${element[0]}.svg`
                            : `img/index/integration/icons/${element[0]}.svg`
                        }
                        alt={element[1]}
                        loading="lazy"
                        style={{
                          width: '64px',
                          objectFit: 'initial',
                          margin: '0 auto',
                        }}
                      />
                      <div>
                        <ImageListItemBar
                          title={element[1]}
                          subtitle={soon.indexOf(element[0]) != -1 ? '(soon)' : ''}
                          position="below"
                          style={{
                            width: '150px',
                            margin: '0 auto',
                            textAlign: 'center',
                          }}
                        />
                      </div>
                    </ImageListItem>
                  </a>
                ))}
              </ImageList>
            </Grid>
          </Grid>
          <Grid item xs={12} className={styles.footer}>
            {' '}
          </Grid>
        </Grid>
        <Grid item md={1} lg={1} xl={1}></Grid>
      </Grid>

      <Grid container>
        <Grid item xs={0.5}></Grid>
        <Grid
          item
          xs={11}
          style={{
            minHeight: '400px',
          }}
          className={styles.vscodeMobile}
        >
          <div className={styles.head}>
            {' '}
            <img src="img/index/header/buttons.svg" style={{ padding: '15px', width: '100px' }} />
          </div>
          <Grid container style={{ height: '87%' }}>
            <Grid item xs={4} className={styles.video} style={{ borderLeft: '2px solid black' }}>
              <span style={{ fontSize: '16px' }}>
                <KeyboardArrowDownIcon className={styles.cardIcons} /> Integrations
              </span>

              <RadioGroup
                aria-labelledby="demo-controlled-radio-buttons-group"
                name="controlled-radio-buttons-group"
                value={currentInt}
                onChange={handleChange}
              >
                <FormControlLabel
                  value="mcu"
                  className={currentInt == 'mcu' ? styles.engineActive : styles.engine}
                  control={<Radio />}
                  label="MCU & IDE"
                />
                <FormControlLabel
                  value="os"
                  className={currentInt == 'os' ? styles.engineActive : styles.engine}
                  control={<Radio />}
                  label="OS & Frameworks"
                />
                <FormControlLabel
                  value="api"
                  className={currentInt == 'api' ? styles.engineActive : styles.engine}
                  control={<Radio />}
                  label="APIs & 
                  integrations"
                />
                <FormControlLabel
                  value="sdk"
                  className={currentInt == 'sdk' ? styles.engineActive : styles.engine}
                  control={<Radio />}
                  label="SDK & 
                  languages"
                />
              </RadioGroup>
            </Grid>
            <Grid item xs={8} lg={9} className={styles.player}>
              <ImageList cols={2} style={{ padding: '10px' }}>
                {currentImageTmp.map((element, index) => (
                  <a href={links[element[0]]} className={styles.imgLink}>
                    <ImageListItem key={index}>
                      <img
                        src={`img/index/integration/icons/${element[0]}.svg`}
                        alt={element[1]}
                        loading="lazy"
                        style={{
                          width: '48px',
                          objectFit: 'initial',
                          margin: '0 auto',
                        }}
                      />
                      <ImageListItemBar
                        title={element[1]}
                        subtitle={soon.indexOf(element[0]) != -1 ? '(soon)' : ''}
                        position="below"
                        style={{
                          width: '150px',
                          margin: '0 auto',
                          textAlign: 'center',
                        }}
                      />
                    </ImageListItem>
                  </a>
                ))}
              </ImageList>
            </Grid>
          </Grid>
          <Grid item xs={12} className={styles.footer}>
            {' '}
          </Grid>
        </Grid>
        <Grid item xs={0.5}></Grid>
      </Grid>

      <Grid container spacing={3}>
        <Grid item xs={2} md={4} lg={4}></Grid>
        <Grid item xs={4} md={4} lg={4}></Grid>
        <Grid item xs={6} md={4} lg={4}>
          <img src="img/index/integration/line-right.svg" className={styles.imgRight} />
        </Grid>
      </Grid>
    </div>
  );
};
export default Integration;
