import React from 'react';
import clsx from 'clsx';
import Translate from '@docusaurus/Translate';
import { ThemeClassNames, useDocsVersion } from '@docusaurus/theme-common';
import Link from '@docusaurus/Link';
export default function DocVersionBadge({ className }) {
  const versionMetadata = useDocsVersion();
  if (versionMetadata.badge) {
    return (
      <Link
        href={
          versionMetadata.version !== 'current'
            ? 'https://github.com/Luos-io/luos_engine/releases/tag/' + versionMetadata.version
            : null
        }
      >
        <span
          className={clsx(
            className,
            ThemeClassNames.docs.docVersionBadge,
            'badge badge--secondary',
          )}
        >
          <Translate
            id="theme.docs.versionBadge.label"
            values={{ versionLabel: versionMetadata.label }}
          >
            {'Version: {versionLabel}'}
          </Translate>
        </span>
      </Link>
    );
  }
  return null;
}
