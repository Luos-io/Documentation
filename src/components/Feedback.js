import React, { useRef, useState } from 'react';
import emailjs from '@emailjs/browser';
import useIsBrowser from '@docusaurus/useIsBrowser';
import CircularProgress from '@mui/material/CircularProgress';
import Box from '@mui/material/Box';
import Button from '@mui/material/Button';

export default function ContactUs() {
  const isBrowser = useIsBrowser();
  const [sending, isSending] = useState(false);
  const form = useRef();
  const [sourcePage, setSourcePage] = useState('');

  const sendEmail = (e) => {
    e.preventDefault();
    if (isBrowser) {
      isSending(true);
      setSourcePage(localStorage.getItem('prevPageContactUs'));
      emailjs
        .sendForm(
          process.env.REACT_APP_EMAIL_JS_SERVICE_ID,
          process.env.REACT_APP_EMAIL_JS_TEMPLATE_ID,
          document.getElementById('feedbackForm'),
          process.env.REACT_APP_EMAIL_JS_USER_ID,
        )
        .then(
          () => {
            document.getElementsByClassName('feedbackForm__response')[0].classList.add('success');
            document.getElementById('response').innerText = 'Your feedback has been sent.';
            localStorage.setItem('prevPageContactUs', '');
          },
          (error) => {
            document.getElementsByClassName('feedbackForm__response')[0].classList.add('error');
            document.getElementById('response').innerText = error.text;
          },
        )
        .finally(() => {
          isSending(false);
          document.getElementById('feedbackForm').reset();
        });
    }
  };

  return (
    <div className="row container-feedback">
      <div className="col-info">
        <div className="row">
          <h1 className="col-info__title">Feedback and concerns</h1>
          <p className="col-info__p">
            You didn't find our docs useful? Give us your feedback so we can help you.
          </p>
        </div>
        <div className="col-info__img__container">
          <img className="col-info__img" src="/img/letter.png" />
        </div>
      </div>
      <div className="col-form">
        <form className="feedbackForm" id="feedbackForm" ref={form} onSubmit={sendEmail}>
          <input name="user_page" className="inputForm" type="hidden" value={sourcePage} />
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
            <input type="text" className="inputForm" placeholder="John Doe" name="user_name" />
          </div>
          <div className="feedbackForm__field">
            <label htmlFor="user_email">Your Email * (to reach you with a solution)</label>
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
