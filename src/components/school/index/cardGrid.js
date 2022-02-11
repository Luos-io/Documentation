import React from 'react';
import Grid from '@mui/material/Grid';
import { Paper } from '@mui/material';
import Chip from '@mui/material/Chip';
import styles from './index.module.css';

export default function CardGrid(data, test) {
  console.log(test);
  const selection = data.selection;
  return (
    <Grid container spacing={2}>
      {selection.map((x, y) => (
        <Grid className={styles.cardContainer} key={y} item xs={12} md={6}>
          <Paper className={styles.card} elevation={1}>
            <img src={`/img/school/${x.img}.svg`} />
            <h2 className={styles.cardTitle}>{x.title}</h2>

            <Grid container spacing={1}>
              <Grid className={styles.cardText} item xs={4}>
                <img
                  className={styles.cardIcons}
                  src="/img/school/category.svg"
                />
                <span>{x.category}</span>
              </Grid>
              <Grid className={styles.cardText} item xs={3}>
                <img className={styles.cardIcons} src="/img/school/clock.svg" />
                <span>{x.toc / 60} Hours</span>
              </Grid>
              <Grid className={styles.cardLastText} item xs={3}>
                <img className={styles.cardIcons} src="/img/school/level.svg" />
                <span>{x.level}</span>
              </Grid>
            </Grid>
            <p className={styles.cardDesc}>
              What will you learn with this tutorial in 3 lines. What will you
              learn with this tutorial in 3 lines. What will you learn with this
              tutorial in 3 lines. What will you learn with this tutorial in 3
              lines.{' '}
            </p>
            <Grid container spacing={1}>
              {x.tags.map((tag, y) => (
                <Grid className={styles.cardTag} key={y} item xs={2}>
                  <Chip label={tag} variant="outlined" />
                </Grid>
              ))}
            </Grid>
          </Paper>
        </Grid>
      ))}
    </Grid>
  );
}
