import React from 'react';
import Link from '@docusaurus/Link';
import styles from './styles.module.css';
import { NavbarSecondaryMenuFiller } from '@docusaurus/theme-common';

function BlogSidebarMobileSecondaryMenu({ sidebar }) {
  return (
    <div>
      <div className={styles.sidebarItemTitle}>
        <Link className={styles.titleLink} to="/blog">
          {sidebar.title}
        </Link>
      </div>
      <ul className="menu__list">
        {sidebar.items.map((item) => (
          <li key={item.permalink} className="menu__list-item">
            <Link
              isNavLink
              to={item.permalink}
              className="menu__link"
              activeClassName="menu__link--active"
            >
              {item.title}
            </Link>
          </li>
        ))}
      </ul>
    </div>
  );
}

export default function BlogSidebarMobile(props) {
  return (
    <NavbarSecondaryMenuFiller
      component={BlogSidebarMobileSecondaryMenu}
      props={props}
    />
  );
}
