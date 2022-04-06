import React from 'react';
import Grid from '@mui/material/Grid';
import { Paper } from '@mui/material';
import styles from './index.module.css';

const cardGrid = (props) => {
  const level = {
    1: 'Beginner',
    2: 'Confirmed',
    3: 'Expert',
  };

  const filters = props.filter;
  let filtered = props.selection;
  Object.keys(filters).forEach((key) => {
    if (filters[key]) {
      filtered = filtered.filter((el) => {
        switch (key) {
          case 'category':
            return el[key] === filters[key];
          case 'toc':
            if (filters[key] === 361) {
              return el[key] >= filters[key];
            }
            return el[key] <= filters[key];

          case 'level':
            return el[key] === parseInt(filters[key]);
          case 'tags':
            // return el[key].includes(filters[key]);
            return filters[key].every((v) => el[key].includes(v));
        }
      });
    }
  });

  return (
    <Grid container spacing={2}>
      {filtered.map((x, y) => (
        <Grid className={styles.cardContainer} key={y} item xs={12} md={4}>
          <a
            href={x.link}
            className={styles.link}
            style={{ textDecoration: 'none' }}
          >
            <Paper className={styles.card} elevation={1}>
              <img
                src={`/img/school/${x.img}.svg`}
                style={{ borderRadius: '4px' }}
              />
              <h2 className={styles.cardTitle}>{x.title}</h2>

              <Grid container spacing={1}>
                <Grid item xs={6}>
                  <div>
                    <span>
                      {' '}
                      <img
                        className={styles.cardIcons}
                        src="/img/school/category.svg"
                      />
                    </span>
                    <span className={styles.levelTxt}>{x.category}</span>
                  </div>
                  <div>
                    <span>
                      {' '}
                      <img
                        className={styles.cardIcons}
                        src="/img/school/clock.svg"
                      />
                    </span>
                    <span className={styles.levelTxt}>
                      {Math.round(x.toc)} Minutes
                    </span>
                  </div>
                </Grid>
                <Grid item xs={6}>
                  <div style={{ width: '100%' }}>
                    <div className={`${styles.difficultyContainer}`}>
                      <span
                        className={`${styles.difficulty} ${level[x.level]}`}
                      ></span>
                      <span>{level[x.level]}</span>
                    </div>
                  </div>
                </Grid>
              </Grid>
              <p className={styles.cardDesc}>{x.desc}</p>
            </Paper>
          </a>
        </Grid>
      ))}
    </Grid>
  );
};

export default cardGrid;
