import React from 'react';
import clsx from 'clsx';
import Translate from '@docusaurus/Translate';
import { ThemeClassNames, useDocsVersion } from '@docusaurus/theme-common';
import Link from '@docusaurus/Link';
import versionList from './version.json';
export default function DocVersionBadge({ className }) {
  const versionMetadata = useDocsVersion();

  const currentVersion =
    versionMetadata.version !== 'current'
      ? versionList[versionMetadata.version]
      : versionMetadata.label;
  if (versionMetadata.badge) {
    return (
      <Link
        href={
          versionMetadata.version !== 'current'
            ? 'https://github.com/Luos-io/luos_engine/releases/tag/' + currentVersion
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
          <Translate id="theme.docs.versionBadge.label" values={{ versionLabel: currentVersion }}>
            {'Version: {versionLabel}'}
          </Translate>
        </span>
      </Link>
    );
  }
  return null;
}
