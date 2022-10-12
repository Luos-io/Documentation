import React from 'react';
import Grid from '@mui/material/Grid';
import { Paper } from '@mui/material';

import styles from './index.module.css';

const ToolsCard = ({ data }) => {
  return (
    <Grid container spacing={2} mb={5}>
      {data.map((x, y) => (
        <Grid className={styles.cardContainer} key={y} item xs={12} md={4}>
          <a
            href={x.link}
            target={x.target !== undefined ? '_blank' : ''}
            className={styles.link}
            style={{ textDecoration: 'none' }}
          >
            <Paper className={styles.card} elevation={1}>
              <img src={x.img} style={{ borderRadius: '4px' }} alt={x.title} />
              <h2 className={styles.cardTitle}>{x.title}</h2>

              <p className={styles.cardDesc}>{x.desc}</p>
            </Paper>
          </a>
        </Grid>
      ))}
    </Grid>
  );
};

export default ToolsCard;
