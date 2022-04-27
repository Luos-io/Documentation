import React, { useState, useEffect } from 'react';
import styles from './integration.module.css';
import Grid from '@mui/material/Grid';
import Button from '@mui/material/Button';
import Radio from '@mui/material/Radio';
import RadioGroup from '@mui/material/RadioGroup';
import FormControlLabel from '@mui/material/FormControlLabel';
import Typography from '@mui/material/Typography';
import Stack from '@mui/material/Stack';
import KeyboardArrowDownIcon from '@mui/icons-material/KeyboardArrowDown';
import { Avatar } from '@mui/material';

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
      ['microros', 'Micro-ROS'],
      ['freertos', 'FreeRTOS'],
    ],
    api: [
      ['simplefoc', 'SimpleFOC'],
      ['freedom', 'Freedom Robotics (soon)'],
      ['zappier', 'Zappier (soon)'],
      ['ifttt', 'IFTTT (soon)'],
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
        <Grid item md={4} lg={4}>
          <h1 className={`${styles.title} ${styles.underline}`}>
            Integrations
          </h1>
        </Grid>
      </Grid>
      <Grid container spacing={3} sx={{ padding: '30px' }}>
        <Grid item md={2} lg={2} xl={3.5}></Grid>
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
              <div>
                {currentImageTmp.map((element, index) => (
                  <div
                    style={{
                      textAlign: 'center',
                      display: 'inline-block',
                      padding: '10px',
                    }}
                  >
                    <Avatar
                      src={`img/index/integration/icons/${element[0]}.svg`}
                      variant="square"
                      key={index}
                      style={{ margin: '0 auto' }}
                    />
                    <Typography className={styles.icoText}>
                      {element[1]}{' '}
                    </Typography>
                  </div>
                ))}
              </div>
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
      <Grid container spacing={3}>
        <Grid item md={8} lg={8}></Grid>
        <Grid item md={4} lg={4}>
          <img src="img/index/right-lines.svg" className={styles.imgRight} />
        </Grid>
      </Grid>
    </div>
  );
};
export default Integration;
