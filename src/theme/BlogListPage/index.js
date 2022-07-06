import React from 'react';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import BlogLayout from '@theme/BlogLayout';
import BlogPostItem from '@theme/BlogPostItem';
import BlogListPaginator from '@theme/BlogListPaginator';
import { PageMetadata, HtmlClassNameProvider, ThemeClassNames } from '@docusaurus/theme-common';
import SearchMetadata from '@theme/SearchMetadata';
import clsx from 'clsx';
import Divider from '@mui/material/Divider';

function BlogListPageMetadata(props) {
  const { metadata } = props;
  const {
    siteConfig: { title: siteTitle },
  } = useDocusaurusContext();
  const { blogDescription, blogTitle, permalink } = metadata;
  const isBlogOnlyMode = permalink === '/';
  const title = isBlogOnlyMode ? siteTitle : blogTitle;
  return (
    <>
      <PageMetadata title={title} description={blogDescription} />
      <SearchMetadata tag="blog_posts_list" />
    </>
  );
}

function BlogListPageContent(props) {
  const { metadata, items, sidebar } = props;
  return (
    <BlogLayout sidebar={sidebar}>
      <h1 style={{ display: 'none' }}>Blog articles about embedded and technology topics</h1>
      {items.map(({ content: BlogPostContent }) => (
        <BlogPostItem
          key={BlogPostContent.metadata.permalink}
          frontMatter={BlogPostContent.frontMatter}
          assets={BlogPostContent.assets}
          metadata={BlogPostContent.metadata}
          truncated={BlogPostContent.metadata.truncated}
        >
          <BlogPostContent />
        </BlogPostItem>
      ))}
      <BlogListPaginator metadata={metadata} />
      <Divider />
      <p>
        The Luos Blog is the perfect place to get information and to keep an eye on many topics:
        embedded and edge systems microservices in hardware devices, robotics Luos 🙂 , on PID
        control in motors. But not only! We will also talk about motors used for robotics and
        electronics broadly. We will try to contribute to your knowledge of the field as an embedded
        developer on questions such as:
        <br />
        <a href="https://www.luos.io/blog/get-to-know-your-dc-motor-how-to-read-a-datasheet">
          How to read component datasheet?
        </a>
        How to make a robot, What are the basics of embedded systems, What is the importance of
        embedded systems, how to reverse a dc motor, etc.
        <br />
        Many projects are waiting for you on the Luos blog with texts, webinars, and{' '}
        <a href="https://www.youtube.com/channel/UCWeIoHVY9Z-04kdwXNtv2FA">videos</a>, so bookmark
        the blog and <a href="https://discord.gg/luos">join our Discord community</a>for even more
        exchanges on your projects! A quick tip, want to get started with our blog? Check out our
        guide that talks about the
        <a href="https://www.luos.io/blog/how-to-lift-a-banana-a-guide-to-motors-in-robotics">
          basics of electric motors for robotic applications
        </a>
        and explains how to choose the right motor for your use and project.
      </p>
    </BlogLayout>
  );
}

export default function BlogListPage(props) {
  return (
    <HtmlClassNameProvider
      className={clsx(ThemeClassNames.wrapper.blogPages, ThemeClassNames.page.blogListPage)}
    >
      <BlogListPageMetadata {...props} />
      <BlogListPageContent {...props} />
    </HtmlClassNameProvider>
  );
}
