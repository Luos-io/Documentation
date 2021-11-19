/**
 * Copyright (c) Facebook, Inc. and its affiliates.
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 */
import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import { useThemeConfig } from '@docusaurus/theme-common';
import useBaseUrl from '@docusaurus/useBaseUrl';
import isInternalUrl from '@docusaurus/isInternalUrl';
import styles from './styles.module.css';
import ThemedImage from '@theme/ThemedImage';
import IconExternalLink from '@theme/IconExternalLink';
import GitHubButton from 'react-github-btn';

function FooterLink({ to, href, label, prependBaseUrlToHref, ...props }) {
  const toUrl = useBaseUrl(to);
  const normalizedHref = useBaseUrl(href, {
    forcePrependBaseUrl: true,
  });
  return (
    <Link
      className="footer__link-item"
      {...(href
        ? {
            href: prependBaseUrlToHref ? normalizedHref : href,
          }
        : {
            to: toUrl,
          })}
      {...props}
    >
      {href && !isInternalUrl(href) ? (
        <span>
          {label}
          <IconExternalLink width="10" />
        </span>
      ) : (
        label
      )}
    </Link>
  );
}

const FooterLogo = ({ sources, alt }) => (
  <ThemedImage className="footer__logo" alt={alt} sources={sources} />
);

function Footer() {
  const { footer } = useThemeConfig();
  const { copyright, links = [], logo = {} } = footer || {};
  const sources = {
    light: useBaseUrl(logo.src),
    dark: useBaseUrl(logo.srcDark || logo.src),
  };

  if (!footer) {
    return null;
  }

  return (
    <footer
      className={clsx('footer', {
        'footer--dark': footer.style === 'dark',
      })}
    >
      <div className="container">
        <div className="joinUsContainer">
          <h3>Join us on</h3>
          <a href="http://bit.ly/JoinLuosDiscord">
            <img src="/img/discord.png" className="rsLogo"></img>
          </a>
          <a href="http://bit.ly/JoinLuosSlack">
            <img src="/img/slack.png" className="rsLogo"></img>
          </a>
          <a href="https://www.reddit.com/r/Luos/">
            <img src="/img/reddit.png" className="rsLogo"></img>
          </a>
          <a href="https://www.linkedin.com/company/luos">
            <img src="/img/linkedin.png" className="rsLogo"></img>
          </a>
        </div>
        {links && links.length > 0 && (
          <div className="row footer__links">
            {links.map((linkItem, i) => (
              <div key={i} className="custom_mobile_col col footer__col">
                {linkItem.title != null ? (
                  <div className="footer__title widget-title">
                    <h4> {linkItem.title}</h4>
                    <span></span>
                  </div>
                ) : null}
                {linkItem.items != null &&
                Array.isArray(linkItem.items) &&
                linkItem.items.length > 0 ? (
                  <ul className="footer__items">
                    {linkItem.items.map((item, key) =>
                      item.html ? (
                        <li
                          key={key}
                          className="footer__item" // Developer provided the HTML, so assume it's safe.
                          // eslint-disable-next-line react/no-danger
                          dangerouslySetInnerHTML={{
                            __html: item.html,
                          }}
                        />
                      ) : (
                        <li key={item.href || item.to} className="footer__item">
                          <FooterLink {...item} />
                        </li>
                      ),
                    )}
                  </ul>
                ) : null}
              </div>
            ))}
          </div>
        )}
        <div className="footer__bottom btn_footer text--center">
          <GitHubButton
            href="https://github.com/luos-io"
            aria-label="Follow @luos-io on GitHub"
            data-size="large"
          >
            Follow @luos-io
          </GitHubButton>

          <GitHubButton
            href="https://github.com/luos-io/Luos"
            data-icon="octicon-star"
            aria-label="Star luos-io/documentation on GitHub"
            data-size="large"
            data-show-count="true"
          >
            Star
          </GitHubButton>
        </div>
        {(logo || copyright) && (
          <div className="footer__bottom text--center">
            {logo && (logo.src || logo.srcDark) && (
              <div className="margin-bottom--sm">
                {logo.href ? (
                  <Link href={logo.href} className={styles.footerLogoLink}>
                    <FooterLogo alt={logo.alt} sources={sources} />
                  </Link>
                ) : (
                  <FooterLogo alt={logo.alt} sources={sources} />
                )}
              </div>
            )}
            {copyright ? (
              <div
                className="footer__copyright" // Developer provided the HTML, so assume it's safe.
                // eslint-disable-next-line react/no-danger
                dangerouslySetInnerHTML={{
                  __html: copyright,
                }}
              />
            ) : null}
          </div>
        )}
      </div>
    </footer>
  );
}

export default Footer;
