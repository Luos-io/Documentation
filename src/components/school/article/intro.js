import React, { useEffect, useState } from 'react';
import { Paper } from '@mui/material';
import Grid from '@mui/material/Grid';
import Chip from '@mui/material/Chip';
import Stack from '@mui/material/Stack';
import styles from './article.module.css';

export const Introduction = () => {
  const keyword = ['test 1', 'test 2', 'test 3'];
  keyword.forEach((element) => {
    console.log(element);
  });
  return (
    <Paper className={styles.introContainer} elevation={1}>
      <Grid className={styles.titleContainer} container>
        <Grid item md={6} xs={12}>
          <h1 className={styles.title}>Title</h1>
        </Grid>
        <Grid item md={6} xs={12}>
          <div className={styles.navigation}>
            <span>
              <img
                className={styles.categoryIcons}
                src="/img/school/category.svg"
              />
            </span>
            <span className={styles.counter}>Category</span>
            <span>
              <img
                className={styles.materialIcons}
                src="/img/school/clock.svg"
              />
            </span>
            <span className={styles.counter}>Time</span>
            <span>
              <img
                className={styles.materialIcons}
                src="/img/school/level.svg"
              />
            </span>
            <span className={styles.lastCounter}>level</span>
          </div>
        </Grid>
      </Grid>
      <h2 className={styles.subtitle}>What will you learn</h2>
      <Stack direction="row" spacing={1} marginBottom={5}>
        {keyword.map((element, index) => (
          <Chip key={index} label={element} />
        ))}
      </Stack>
      <p>
        We will use one of these nodes as a gate and the other as an application
        node. The second node will host a bootloader, and you will be able to
        update its firmware through the gate. You need an USB shield to connect
        to the first node to complete this tutorial.
      </p>
    </Paper>
  );
};

export default Introduction;
