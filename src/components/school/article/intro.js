import React from 'react';
import { Paper } from '@mui/material';
import Grid from '@mui/material/Grid';
import Chip from '@mui/material/Chip';
import Stack from '@mui/material/Stack';
import styles from './article.module.css';
import Requirement from './requirement';

export const Introduction = (props) => {
  const keyword = props.tags;
  return (
    <div style={{ marginTop: '-65px' }}>
      <Paper className={styles.introContainer} elevation={1}>
        <Grid className={styles.titleContainer} container>
          <Grid item md={7} xs={12}>
            <h1 className={styles.title}>{props.title}</h1>
          </Grid>
          <Grid item md={5} xs={12}>
            <div className={styles.navigation}>
              <span>
                <img
                  className={styles.categoryIcons}
                  src="/img/school/category.svg"
                />
              </span>
              <span className={styles.counter}>{props.category}</span>
              <span>
                <img
                  className={styles.materialIcons}
                  src="/img/school/clock.svg"
                />
              </span>
              <span className={styles.lastCounter}>{props.time}</span>
            </div>
            <div className={`${styles.difficultyContainer}`}>
              <span className={`${styles.difficulty} ${props.level}`}></span>
              <span>{props.level}</span>
            </div>
          </Grid>
        </Grid>
        <h2 className={styles.subtitle}>What will you learn</h2>
        <p>{props.desc}</p>
        <Stack direction="row" spacing={1} marginBottom={2}>
          {keyword.map((element, index) => (
            <Chip
              key={index}
              label={element}
              className={styles.chipCustom}
              size="small"
            />
          ))}
        </Stack>

        <Grid container spacing={5}>
          <Grid item md={6} xs={12}>
            <Requirement
              title="Pre-requisites"
              list={props.requierements}
              color="#f5f5f5"
            />
          </Grid>
          <Grid item md={6} xs={12}>
            <Requirement
              title="Supported Hardware"
              list={props.ressources}
              shortList={!!props.shortList}
              shortListSize={props.shortListSize}
              color="#f5f5f5"
            />
          </Grid>
        </Grid>
      </Paper>
      <Grid container mt={3}>
        <Grid item xs={12}>
          <Requirement title="Summary" color="#FFFFFF" list={props.summary} />
        </Grid>
      </Grid>
    </div>
  );
};

export default Introduction;
