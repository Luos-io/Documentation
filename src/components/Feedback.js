import React, { useRef, useState } from 'react';
import emailjs from 'emailjs-com';
import useIsBrowser from '@docusaurus/useIsBrowser';
import CircularProgress from '@mui/material/CircularProgress';
import Box from '@mui/material/Box';
import Button from '@mui/material/Button';

export default function ContactUs(props) {
  const isBrowser = useIsBrowser();
  const [sending, isSending] = useState(false);
  const form = useRef();
  let sourcePage;
  let sendEmail = () => {};

  if (isBrowser) {
    sourcePage = localStorage.getItem('prevPageContactUs');
    let element = document.getElementById('response');
    let elementContainer = document.getElementsByClassName(
      'feedbackForm__response',
    );

    sendEmail = (e) => {
      isSending(true);
      e.preventDefault();
      let form = document.getElementById('feedbackForm');
      emailjs
        .sendForm(
          process.env.REACT_APP_EMAIL_JS_SERVICE_ID,
          process.env.REACT_APP_EMAIL_JS_TEMPLATE_ID,
          form,
          process.env.REACT_APP_EMAIL_JS_USER_ID,
        )
        .then(
          () => {
            isSending(false);
            elementContainer[0].classList.add('success');
            element.innerText = 'Your feedback has been sent.';
            localStorage.setItem('prevPageContactUs', '');
            form.reset();
          },
          (error) => {
            isSending(false);
            elementContainer[0].classList.add('error');
            element.innerText = error.text;
            form.reset();
          },
        );
    };
  }

  return (
    <div className="row container-feedback">
      <div className="col-info">
        <div className="row">
          <h1 className="col-info__title">Feedback and concerns</h1>
          <p className="col-info__p">
            You didn't find our docs useful? Give us your feedback so we can
            help you.
          </p>
        </div>
        <div className="col-info__img__container">
          <img className="col-info__img" src="/img/letter.png" />
        </div>
      </div>
      <div className="col-form">
        <form
          className="feedbackForm"
          id="feedbackForm"
          ref={form}
          onSubmit={sendEmail}
        >
          <input
            name="user_page"
            className="inputForm"
            type="hidden"
            value={sourcePage}
          />
          <div className="feedbackForm__field">
            <label htmlFor="user_object">What's your headache? *</label>
            <input
              className="inputForm"
              type="text"
              required
              placeholder="I miss information about Luos services..."
              name="user_object"
            />
          </div>
          <div className="feedbackForm__field">
            <label htmlFor="message">In more details</label>
            <textarea
              className="inputForm"
              rows="9"
              name="message"
              placeholder="How should I configure my services in order to get my rocket off the ground?"
            />
          </div>
          <div className="feedbackForm__field">
            <label htmlFor="user_name">Your Full Name</label>
            <input
              type="text"
              className="inputForm"
              placeholder="John Doe"
              name="user_name"
            />
          </div>
          <div className="feedbackForm__field">
            <label htmlFor="user_email">
              Your Email * (to reach you with a solution)
            </label>
            <input
              className="inputForm"
              required
              type="email"
              placeholder="john@luos.io"
              name="user_email"
            />
          </div>
          <Button type="submit" className="send" variant="contained">
            {sending ? (
              <Box sx={{ display: 'flex' }}>
                <CircularProgress size={24} sx={{ color: '#282D3F' }} />
              </Box>
            ) : (
              'Send'
            )}
          </Button>
        </form>
        <div className="feedbackForm__response">
          <p id="response"></p>
        </div>
      </div>
    </div>
  );
}
