import React from 'react';
import Grid from '@mui/material/Grid';
import { Paper } from '@mui/material';
import Moment from 'moment';

import styles from './index.module.css';

const cardGrid = (props) => {
  let filtered = props.selection;
  const allArticles = props.selection;
  const filters = props.filter;

  const displayedList =
    filters.tags.length > 0 ? filtered.filter((el) => el.tags.includes(filters.tags)) : allArticles;

  return (
    <Grid container spacing={2}>
      {displayedList.map((x, y) => (
        <Grid className={styles.cardContainer} key={y} item xs={12} md={4}>
          <a href={x.link} className={styles.link} style={{ textDecoration: 'none' }}>
            <Paper className={styles.card} elevation={1}>
              <img src={x.img} style={{ borderRadius: '4px' }} alt={x.title} />
              <h2 className={styles.cardTitle}>{x.title}</h2>

              <Grid container spacing={1}>
                <Grid item xs={12}>
                  <Grid container spacing={1} alignItems="center">
                    <Grid item xs={2}>
                      <img className="avatar__photo" src={x.author_img} alt="author" />
                    </Grid>
                    <Grid item xs={8}>
                      <span className={styles.levelTxt}>{x.author}</span>
                    </Grid>
                  </Grid>
                  {x.author_2 !== undefined ? (
                    <Grid container spacing={1} mt={1} alignItems="center">
                      <Grid item xs={2}>
                        <img className="avatar__photo" src={x.author_img_2} alt="author" />
                      </Grid>
                      <Grid item xs={8}>
                        <span className={styles.levelTxt}>{x.author_2}</span>
                      </Grid>
                    </Grid>
                  ) : null}

                  <span className={styles.date}>{Moment(x.date).format('MMM Do, YYYY')}</span>
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
