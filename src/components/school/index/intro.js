import React, { useEffect, useState } from 'react';
import { Paper } from '@mui/material';
import Grid from '@mui/material/Grid';
import FormControl from '@mui/material/FormControl';
import Select from '@mui/material/Select';
import InputLabel from '@mui/material/InputLabel';
import MenuItem from '@mui/material/MenuItem';
import ToggleButton from '@mui/material/ToggleButton';
import ToggleButtonGroup from '@mui/material/ToggleButtonGroup';
import CardGrid from './cardGrid';
import styles from './index.module.css';
import data from './data/dataIntro.json';

const Intro = () => {
  const [filters, setfilters] = useState({
    toc: '',
    tags: '',
    hardware: '',
    category: '',
    lvl: '',
  });

  const handleToc = (event, newLevel) => {
    setfilters({
      ...filters,
      toc: newLevel.props.value,
    });
  };

  const handleTopic = (event, newLevel) => {
    setfilters({
      ...filters,
      tags: newLevel.props.value,
    });
  };

  const handleHardware = (event, newLevel) => {
    setfilters({
      ...filters,
      hardware: newLevel.props.value,
    });
  };

  const handleCategory = (event, newLevel) => {
    console.log(newLevel.props.value);
    setfilters({
      ...filters,
      category: newLevel.props.value,
    });
  };

  const handleLevel = (event, newLevel) => {
    setfilters({
      ...filters,
      lvl: newLevel,
    });
  };

  return (
    <div>
      <Paper className={styles.introContainer} elevation={1}>
        {/* Title & Text container */}
        <Grid container spacing={2}>
          <Grid item xs={8}>
            <h2>{data.title}</h2>
          </Grid>
          <Grid item xs={4}>
            {/* TODO set value with JSON */}
            <span className={styles.stats}>X tutorials | X Hours</span>
          </Grid>
          <Grid item xs={10}>
            <p className={styles.introText}>{data.introText}</p>
          </Grid>
        </Grid>
        {/* Filter container */}
        <Grid container spacing={2}>
          <Grid item xs={9}>
            <FormControl sx={{ m: 1, minWidth: 100 }}>
              <InputLabel id="toc-label">Time to complete</InputLabel>
              <Select
                labelId="toc-label"
                id="toc"
                value={filters.toc}
                className={styles.filterBtn}
                sx={{ width: '200px' }}
                onChange={handleToc}
              >
                {data.filters.toc.map((filter, index) => (
                  <MenuItem value={filter.id} key={index}>
                    {filter.label}
                  </MenuItem>
                ))}
              </Select>
            </FormControl>
            <FormControl sx={{ m: 1, minWidth: 120 }}>
              <InputLabel id="topic-label">Topics(s)</InputLabel>
              <Select
                value={filters.tags}
                className={styles.filterBtn}
                labelId="topic-label"
                onChange={handleTopic}
              >
                {data.filters.tags.map((label, index) => (
                  <MenuItem value={label} key={index}>
                    {label}
                  </MenuItem>
                ))}
              </Select>
            </FormControl>
            <FormControl sx={{ m: 1, minWidth: 120 }}>
              <InputLabel id="hardware-label">Hardware</InputLabel>
              <Select
                value={filters.hardware}
                className={styles.filterBtn}
                labelId="hardware-label"
                onChange={handleHardware}
              >
                {data.filters.hardware.map((label, index) => (
                  <MenuItem value={label} key={index}>
                    {label}
                  </MenuItem>
                ))}
              </Select>
            </FormControl>
            <FormControl sx={{ m: 1, minWidth: 120 }}>
              <InputLabel id="category-label">Category</InputLabel>
              <Select
                value={filters.category}
                className={styles.filterBtn}
                labelId="category-label"
                onChange={handleCategory}
              >
                {data.filters.category.map((label, index) => (
                  <MenuItem value={label} key={index}>
                    {label}
                  </MenuItem>
                ))}
              </Select>
            </FormControl>
          </Grid>
          <Grid item xs={3}>
            <ToggleButtonGroup
              className={styles.lvlBtn}
              color="success"
              value={filters.lvl}
              exclusive
              onChange={handleLevel}
              aria-label="text alignment"
            >
              <ToggleButton value="1" aria-label="left aligned">
                Beginner
              </ToggleButton>
              <ToggleButton value="2" aria-label="centered">
                Confirmed
              </ToggleButton>
              <ToggleButton value="3" aria-label="right aligned">
                Expert
              </ToggleButton>
            </ToggleButtonGroup>
          </Grid>
        </Grid>
      </Paper>

      <CardGrid selection={data.tuto} filter={filters} />
    </div>
  );
};

export default Intro;
