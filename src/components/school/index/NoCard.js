import React from 'react';
import Paper from '@mui/material/Paper';
import Grid from '@mui/material/Grid';
import Styles from './error.module.css';

export default function Error403() {
  return (
    <Grid container spacing={2}>
      <Grid item xs={2} />
      <Grid item xs={8} sx={{ paddingTop: '10px !important' }}>
        <Paper elevation={3} className={Styles.error}>
          <div className={Styles.container}>
            <img
              src="/img/school/close.svg"
              alt="Luos network"
              width="150"
              height="150"
            />
            <h1>Oops</h1>
            <p>There is no course with this filters.</p>
          </div>
        </Paper>
      </Grid>
      <Grid item xs={2} />
    </Grid>
  );
}
