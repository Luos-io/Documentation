import React, { useState } from 'react';
import styles from './integration.module.css';
import Grid from '@mui/material/Grid';
import Button from '@mui/material/Button';
import Radio from '@mui/material/Radio';
import RadioGroup from '@mui/material/RadioGroup';
import FormControlLabel from '@mui/material/FormControlLabel';
import Typography from '@mui/material/Typography';
import KeyboardArrowDownIcon from '@mui/icons-material/KeyboardArrowDown';
import { Avatar } from '@mui/material';
import ImageList from '@mui/material/ImageList';
import ImageListItem from '@mui/material/ImageListItem';
import ImageListItemBar from '@mui/material/ImageListItemBar';

// DO NOT REMOVE WIP INTEGRATION BLOCK

const Integration = (props) => {
  const img = {
    mcu: [
      ['esp', 'ESP32'],
      ['stm', 'STM32'],
      ['microship', 'Microship'],
      ['raspberry', 'Raspberry Pi'],
      ['arduino', 'Arduino'],
      ['pio', 'Platform IO'],
      ['eclipse', 'Eclipse'],
      ['vscodeico', 'VS Code'],
    ],
    os: [
      ['ros', 'ROS-1 & ROS-2'],
      ['freertos', 'FreeRTOS'],
      ['microros', 'Micro-ROS'],
    ],
    api: [
      ['simplefoc', 'SimpleFOC'],
      ['zappier', 'Zappier (soon)'],
      ['ifttt', 'IFTTT (soon)'],
      ['freedom', 'Freedom Robotics (soon)'],
    ],
    sdk: [
      ['c', 'C/C++'],
      ['python', 'Python'],
      ['js', 'Javascript'],
      ['ts', 'Typescript'],
    ],
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
        <Grid item md={4} lg={4}>
          {' '}
          <img src="img/index/left-lines.svg" className={styles.imgLeft} />
        </Grid>
        <Grid item xs={12} md={4} lg={4}>
          <h1 className={`${styles.title} ${styles.underline}`}>
            Integrations
          </h1>
        </Grid>
      </Grid>
      <Grid
        container
        spacing={3}
        sx={{ padding: '30px' }}
        className={styles.mobileNone}
      >
        <Grid item xs={1} md={2} lg={2} xl={3.5}></Grid>
        <Grid
          item
          md={8}
          lg={8}
          xl={5}
          style={{
            borderBottom: '15px solid #81c8be',
            minHeight: '400px',
          }}
          className={styles.vscode}
        >
          <div className={styles.head}>
            {' '}
            <img
              src="img/index/header/buttons.svg"
              style={{ padding: '15px' }}
            />
          </div>
          <Grid container style={{ height: '87%' }}>
            <Grid item md={1} lg={1} className={styles.docs}>
              <img src="img/index/integration/docs.svg" />
              <img src="img/index/integration/zoom.svg" />
              <img src="img/index/integration/luos.svg" />
            </Grid>
            <Grid item md={2} lg={2} className={styles.video}>
              <span>
                <KeyboardArrowDownIcon className={styles.cardIcons} />{' '}
                Integrations
              </span>

              <RadioGroup
                aria-labelledby="demo-controlled-radio-buttons-group"
                name="controlled-radio-buttons-group"
                value={currentInt}
                onChange={handleChange}
              >
                <FormControlLabel
                  value="mcu"
                  className={
                    currentInt == 'mcu' ? styles.engineActive : styles.engine
                  }
                  control={<Radio />}
                  label="MCU & IDE"
                />
                <FormControlLabel
                  value="os"
                  className={
                    currentInt == 'os' ? styles.engineActive : styles.engine
                  }
                  control={<Radio />}
                  label="OS & Frameworks"
                />
                <FormControlLabel
                  value="api"
                  className={
                    currentInt == 'api' ? styles.engineActive : styles.engine
                  }
                  control={<Radio />}
                  label="APIs & 
                  integrations"
                />
                <FormControlLabel
                  value="sdk"
                  className={
                    currentInt == 'sdk' ? styles.engineActive : styles.engine
                  }
                  control={<Radio />}
                  label="SDK & 
                  languages"
                />
              </RadioGroup>
            </Grid>
            <Grid item md={9} lg={9} className={styles.player}>
              <ImageList cols={4} style={{ padding: '15px' }}>
                {currentImageTmp.map((element, index) => (
                  <ImageListItem key={index}>
                    <img
                      src={`img/index/integration/icons/${element[0]}.svg`}
                      alt={element[1]}
                      loading="lazy"
                      style={{ width: '64px' }}
                    />
                    <ImageListItemBar title={element[1]} position="below" />
                  </ImageListItem>
                ))}
              </ImageList>
            </Grid>
          </Grid>
        </Grid>

        <div className={styles.btnContainer}>
          <Button variant="contained" className={styles.pinkBtn}>
            Learn more about integrations
          </Button>
        </div>
        <Grid item md={2} lg={2} xl={3.5}></Grid>
      </Grid>

      <Grid container>
        <Grid item xs={0.5}></Grid>
        <Grid
          item
          xs={11}
          style={{
            borderBottom: '15px solid #81c8be',
            minHeight: '400px',
          }}
          className={styles.vscodeMobile}
        >
          <div className={styles.head}>
            {' '}
            <img
              src="img/index/header/buttons.svg"
              style={{ padding: '15px' }}
            />
          </div>
          <Grid container style={{ height: '87%' }}>
            <Grid
              item
              xs={4}
              className={styles.video}
              style={{ borderLeft: '2px solid black' }}
            >
              <span style={{ fontSize: '16px' }}>
                <KeyboardArrowDownIcon className={styles.cardIcons} />{' '}
                Integrations
              </span>

              <RadioGroup
                aria-labelledby="demo-controlled-radio-buttons-group"
                name="controlled-radio-buttons-group"
                value={currentInt}
                onChange={handleChange}
              >
                <FormControlLabel
                  value="mcu"
                  className={
                    currentInt == 'mcu' ? styles.engineActive : styles.engine
                  }
                  control={<Radio />}
                  label="MCU & IDE"
                />
                <FormControlLabel
                  value="os"
                  className={
                    currentInt == 'os' ? styles.engineActive : styles.engine
                  }
                  control={<Radio />}
                  label="OS & Frameworks"
                />
                <FormControlLabel
                  value="api"
                  className={
                    currentInt == 'api' ? styles.engineActive : styles.engine
                  }
                  control={<Radio />}
                  label="APIs & 
                  integrations"
                />
                <FormControlLabel
                  value="sdk"
                  className={
                    currentInt == 'sdk' ? styles.engineActive : styles.engine
                  }
                  control={<Radio />}
                  label="SDK & 
                  languages"
                />
              </RadioGroup>
            </Grid>
            <Grid item xs={8} lg={9} className={styles.player}>
              <ImageList cols={2} style={{ padding: '10px' }}>
                {currentImageTmp.map((element, index) => (
                  <ImageListItem key={index}>
                    <img
                      src={`img/index/integration/icons/${element[0]}.svg`}
                      alt={element[1]}
                      loading="lazy"
                      style={{ width: '48px' }}
                    />
                    <ImageListItemBar title={element[1]} position="below" />
                  </ImageListItem>
                ))}
              </ImageList>
            </Grid>
          </Grid>
        </Grid>
        <Grid item xs={0.5}></Grid>

        <div className={styles.btnContainerMobile}>
          <Button variant="contained" className={styles.pinkBtn}>
            Learn more about integrations
          </Button>
        </div>
      </Grid>

      <Grid container spacing={3}>
        <Grid item xs={8} md={8} lg={8}></Grid>
        <Grid item xs={4} md={4} lg={4}>
          <img src="img/index/right-lines.svg" className={styles.imgRight} />
        </Grid>
      </Grid>
    </div>
  );
};
export default Integration;
