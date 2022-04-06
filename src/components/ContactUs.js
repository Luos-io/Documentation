import React from "react";
import useIsBrowser from "@docusaurus/useIsBrowser";

export const ContactUs = (props) => {
  const isBrowser = useIsBrowser();
  if (isBrowser) {
    localStorage.setItem("prevPageContactUs", props.pageName);
  }
  return (
    <div className="contactUs">
      <a href="/feedbacks/send" target="_blank">
        You can't find what your're looking for? »
      </a>
    </div>
  );
};
export default ContactUs;
