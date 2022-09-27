import React, { useState } from 'react';
import { Paper } from '@mui/material';
import clsx from 'clsx';
import Box from '@mui/material/Box';
import Chip from '@mui/material/Chip';
import Grid from '@mui/material/Grid';
import BlogLayout from '@theme/BlogLayout';
import FormControl from '@mui/material/FormControl';
import Select from '@mui/material/Select';
import InputLabel from '@mui/material/InputLabel';
import MenuItem from '@mui/material/MenuItem';
import { PageMetadata, HtmlClassNameProvider, ThemeClassNames } from '@docusaurus/theme-common';
import CardGrid from './card';
import styles from './index.module.css';
import data from './data.json';

function BlogListPageMetadata(props) {
  const title = 'Luos blog: embedded, edge, microservices and much more...';
  const blogDescription =
    'Get information and keep an eye on many topics: embedded and edge systems, microservices, robots, electronics, cyber-physical systems etc...';
  return (
    <>
      <PageMetadata title={title} description={blogDescription} />
    </>
  );
}

export default function BlogListPage(props) {
  const [filters, setfilters] = useState({
    tags: [],
  });

  const handleFilter = (newLevel, filterName) => {
    if (newLevel.indexOf('') !== -1) {
      setfilters({
        tags: [],
      });
    } else {
      setfilters({
        ...filters,
        [filterName]: newLevel,
      });
    }
  };

  let tagList = [];
  data.articles.forEach((article) => {
    article.tags.forEach((tag) => {
      tagList.indexOf(tag) === -1 ? tagList.push(tag) : null;
    });
  });

  return (
    <HtmlClassNameProvider
      className={clsx(ThemeClassNames.wrapper.blogPages, ThemeClassNames.page.blogListPage)}
    >
      <BlogListPageMetadata {...props} />
      <BlogLayout>
        <div>
          <Paper className={styles.introContainer} elevation={1}>
            {/* Title & Text container */}
            <Grid container spacing={2}>
              <Grid item xs={8}>
                <h1>Blog</h1>
              </Grid>
              <Grid item xs={4}>
                {/* TODO set value with JSON */}
              </Grid>
              <Grid item xs={10}>
                <p className={styles.introText}>
                  Get information and keep an eye on many topics: embedded and edge systems,
                  microservices, robots, electronics, cyber-physical systems and much more...
                </p>
              </Grid>
            </Grid>
            {/* Filter container */}
            <Grid container spacing={2}>
              <Grid itme xs={12}>
                <FormControl sx={{ m: 1, minWidth: 120 }} size="small">
                  <InputLabel id="topic-label" className={styles.select}>
                    Tags
                  </InputLabel>
                  <Select
                    value={filters.tags}
                    label="Tags"
                    labelId="topic-label"
                    onChange={(e) => {
                      handleFilter(e.target.value, 'tags');
                    }}
                    multiple
                    renderValue={(selected) => (
                      <Box sx={{ display: 'flex', flexWrap: 'wrap', gap: 0.5 }}>
                        {selected.map((value) => (
                          <Chip key={value} label={value} />
                        ))}
                      </Box>
                    )}
                  >
                    <MenuItem value="">All</MenuItem>
                    {tagList.map((label, index) => (
                      <MenuItem value={label} key={index}>
                        {label}
                      </MenuItem>
                    ))}
                  </Select>
                </FormControl>
              </Grid>
            </Grid>
          </Paper>

          <CardGrid selection={data.articles} filter={filters} />
        </div>
      </BlogLayout>
    </HtmlClassNameProvider>
  );
}
