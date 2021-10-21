import React from 'react';

export default function ContactUs (props) {
    localStorage.setItem('prevPageContactUs', props.pageName);
    return (<div className="contactUs">
        <a href="/feedbacks/send">You can't find what your're looking for ? »</a>
    </div>)
}