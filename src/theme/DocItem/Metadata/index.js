import React from 'react';
import { PageMetadata } from '@docusaurus/theme-common';
import { useDoc } from '@docusaurus/theme-common/internal';
export default function DocItemMetadata() {
  const { metadata, frontMatter, assets } = useDoc();
  return (
    <PageMetadata
      title={
        metadata.version === 'current'
          ? metadata.title
          : `${metadata.title} for version ${metadata.version}`
      }
      description={
        metadata.version === 'current'
          ? metadata.description
          : `${metadata.description} for version ${metadata.version}`
      }
      keywords={frontMatter.keywords}
      image={assets.image ?? frontMatter.image}
    />
  );
}
