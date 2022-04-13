import React from 'react';
import styles from './integration.module.css';
import Grid from '@mui/material/Grid';

const Integration = (props) => {
  const integrations = [
    {
      name: 'Eclipse',
      img: 'eclipse.svg',
    },
    {
      name: 'VS Code',
      img: 'vscodeico.svg',
    },
    {
      name: 'PlatformIO',
      img: 'pio.svg',
    },
    {
      name: 'ESP32',
      img: 'esp.svg',
    },
    {
      name: 'STM32',
      img: 'stm.svg',
    },
    {
      name: 'Microship',
      img: 'microship.svg',
    },
    {
      name: 'Arduino',
      img: 'arduino.svg',
    },
    {
      name: 'Raspberry PI',
      img: 'raspberry.svg',
    },
    {
      name: 'Mirco-ROS',
      img: 'microros.svg',
    },
    {
      name: 'FreeRTOS',
      img: 'freertos.svg',
    },
    {
      name: 'SimpleFOC',
      img: 'simplefoc.svg',
    },
    {
      name: 'Python',
      img: 'python.svg',
    },
    {
      name: 'C/C++',
      img: 'c.svg',
    },
    {
      name: 'Freedom Robotics',
      img: 'freedom.svg',
    },
    {
      name: 'ROS',
      img: 'ros.svg',
    },
  ];
  return (
    <div className={styles.container}>
      <Grid container spacing={3}>
        <Grid item md={4} className={styles.leftLines}></Grid>
        <Grid item md={4}>
          <h1 className={`${styles.title} ${styles.underline}`}>
            Integrations
          </h1>
        </Grid>
      </Grid>
      <Grid container spacing={3} sx={{ padding: '30px' }}>
        <Grid item md={2}></Grid>
        <Grid item md={8} className={styles.integrations}>
          <Grid container sx={{ marginLeft: '30px', marginTop: ' 30px' }}>
            {integrations.map((item, i) => (
              <Grid
                item
                md={3}
                sx={{ marginBottom: '15px', textAlign: 'center' }}
              >
                <img src={`/img/index/${item.img}`} className={styles.img} />
                <span className={styles.label}>{item.name}</span>
              </Grid>
            ))}
          </Grid>
        </Grid>
        <Grid item md={2}></Grid>
      </Grid>
      <Grid container spacing={3}>
        <Grid item md={8}></Grid>
        <Grid item md={4} className={styles.rightLines}></Grid>
      </Grid>
    </div>
  );
};
export default Integration;
