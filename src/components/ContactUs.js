import React from 'react';
import useIsBrowser from '@docusaurus/useIsBrowser';

export const ContactUs = (props) => {
  const isBrowser = useIsBrowser();
  if (isBrowser) {
    localStorage.setItem('prevPageContactUs', props.pageName);
  }
  return (
    <div className="contactUs">
      A feedback? Let's discuss on&nbsp;
      <a href="https://discord.gg/luos" target="_blank">
        Discord
      </a>
      &nbsp;|&nbsp;
      <a
        href="https://github.com/Luos-io/Documentation/issues/new?assignees=Simonbdy&labels=documentation&template=documentation-issue-or-suggestion.md&title=%5BDOC%5D"
        target="_blank"
      >
        Github
      </a>
    </div>
  );
};
export default ContactUs;
