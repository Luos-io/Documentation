import React from 'react';
import clsx from 'clsx';
import DocPaginator from '@theme/DocPaginator';
import DocVersionBanner from '@theme/DocVersionBanner';
import DocVersionBadge from '@theme/DocVersionBadge';
import TOC from '@theme/TOC';
import TOCCollapsible from '@theme/TOCCollapsible';
import Heading from '@theme/Heading';
import {
  PageMetadata,
  HtmlClassNameProvider,
  ThemeClassNames,
  useWindowSize,
} from '@docusaurus/theme-common';
import DocBreadcrumbs from '@theme/DocBreadcrumbs';
import MDXContent from '@theme/MDXContent';
import ContactUs from '@site/src/components/ContactUs';
import styles from './styles.module.css';

function DocItemMetadata(props) {
  const { content: DocContent } = props;
  const { metadata, frontMatter, assets } = DocContent;
  const { keywords } = frontMatter;
  const { description, title } = metadata;
  const image = assets.image ?? frontMatter.image;

  return (
    <PageMetadata
      {...{
        title: metadata.version === 'current' ? title : `${title} for version ${metadata.version}`,
        description:
          metadata.version === 'current'
            ? description
            : `${description} for version ${metadata.version}`,
        keywords,
        image,
      }}
    />
  );
}

function DocItemContent(props) {
  const { content: DocContent } = props;
  const { metadata, frontMatter } = DocContent;
  const {
    hide_title: hideTitle,
    hide_table_of_contents: hideTableOfContents,
    toc_min_heading_level: tocMinHeadingLevel,
    toc_max_heading_level: tocMaxHeadingLevel,
  } = frontMatter;
  const { permalink, title } = metadata; // We only add a title if:
  // - user asks to hide it with front matter
  // - the markdown content does not already contain a top-level h1 heading

  const shouldAddTitle = !hideTitle && typeof DocContent.contentTitle === 'undefined';
  const windowSize = useWindowSize();
  const canRenderTOC = !hideTableOfContents && DocContent.toc && DocContent.toc.length > 0;
  const renderTocDesktop = canRenderTOC && (windowSize === 'desktop' || windowSize === 'ssr');

  const regex = /tutorials/g;
  const found = permalink.match(regex);

  let next = {
    permalink: metadata.next.permalink,
    title: metadata.next.title,
  };

  if (metadata.frontMatter.nextUrl) {
    next = {
      permalink: metadata.frontMatter.nextUrl,
      title: metadata.frontMatter.nextTitle,
    };
  }

  return (
    <div className="row">
      <div
        className={clsx(
          'col',
          {
            [styles.docItemCol]: !hideTableOfContents && !found,
            [styles.docItemColTuto]: !hideTableOfContents && found,
          },
          'custom_mobile_col',
        )}
      >
        <DocVersionBanner />
        <div className={styles.docItemContainer}>
          <article>
            <DocBreadcrumbs />
            <DocVersionBadge />

            {canRenderTOC && (
              <TOCCollapsible
                toc={DocContent.toc}
                minHeadingLevel={tocMinHeadingLevel}
                maxHeadingLevel={tocMaxHeadingLevel}
                className={clsx(ThemeClassNames.docs.docTocMobile, styles.tocMobile)}
              />
            )}

            <div className={clsx(ThemeClassNames.docs.docMarkdown, 'markdown')}>
              {/*
               Title can be declared inside md content or declared through
               front matter and added manually. To make both cases consistent,
               the added title is added under the same div.markdown block
               See https://github.com/facebook/docusaurus/pull/4882#issuecomment-853021120
               */}
              {shouldAddTitle && (
                <header>
                  <Heading as="h1">{title}</Heading>
                </header>
              )}
              <MDXContent>
                <DocContent />
                <ContactUs />
              </MDXContent>
            </div>
          </article>

          <DocPaginator previous={metadata.previous} next={next} />
        </div>
      </div>
      {renderTocDesktop && (
        <div className={found ? 'col col--4' : 'col col--3'}>
          <TOC
            toc={DocContent.toc}
            minHeadingLevel={tocMinHeadingLevel}
            maxHeadingLevel={tocMaxHeadingLevel}
            className={ThemeClassNames.docs.docTocDesktop}
            link={DocContent.metadata.permalink}
            parentName={DocContent.metadata.sourceDirName}
          />
        </div>
      )}
    </div>
  );
}

export default function DocItem(props) {
  const docHtmlClassName = `docs-doc-id-${props.content.metadata.unversionedId}`;
  return (
    <HtmlClassNameProvider className={docHtmlClassName}>
      <DocItemMetadata {...props} />
      <DocItemContent {...props} />
    </HtmlClassNameProvider>
  );
}
