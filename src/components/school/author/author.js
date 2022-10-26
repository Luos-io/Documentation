import React from 'react';
import { Paper } from '@mui/material';
import Grid from '@mui/material/Grid';
import Avatar from '@mui/material/Avatar';

import styles from './style.module.css';
import authorList from '../index/data/authors';

const author = ({ name }) => {
  const author = authorList[name];

  return (
    <Paper elevation={2} mt={4} mb={4} className={styles.authorContainer}>
      {/* <h3 className={styles.aboutTheAuthor}>About the Author:</h3> */}
      <Grid container spacing={2}>
        <Grid item md={9}>
          <h3 className={styles.author}>The author: {author.name}</h3>
          <span className={styles.authorDesc}>{author.job}</span>
          <hr className={styles.separator} />
        </Grid>
        <Grid item md={3}>
          <Avatar
            alt={author.name !== 'nicoR' ? author.name : 'N'}
            src={`/assets/images/tutorials/school/authors/${author.img}`}
            sx={{ width: 56, height: 56, float: 'right' }}
          />
        </Grid>
      </Grid>
      <p className={styles.authorText}>{author.desc}</p>
    </Paper>
  );
};

export default author;
