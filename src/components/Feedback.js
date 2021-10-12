import React, { useRef } from 'react';
import emailjs from 'emailjs-com';

export default function ContactUs (props) {
  const form = useRef();
  let sourcePage = localStorage.getItem('prevPageContactUs');
  console.log(sourcePage);
  let element = document.getElementById('response');
  let elementContainer = document.getElementsByClassName('feedbackForm__response');

  const sendEmail = (e) => {
    e.preventDefault();
    elementContainer[0].classList.add('success');
    element.innerText = "Your feedback has been sent.";
    console.log(form.current)
   
    emailjs.sendForm('service_qn8n7j6', 'template_gfbvg8r', form.current, 'user_k5CBCBc8BH5d5jsoNZ8bH')
      .then((result) => {
          elementContainer[0].classList.add('success');
          element.innerText = "Your feedback has been sent.";
          localStorage.setItem('prevPageContactUs', '');
      }, (error) => {
          elementContainer[0].classList.add('error');
          element.innerText = error.text;
      });
  };

  return (
    <div className="row container-feedback">
      <div className="col-info">
        <div className="row">
          <h1 className="col-info__title">Feedback and concerns</h1>
          <p className="col-info__p">You didn't find our docs useful ? Give us your feedback so we can help you.</p>
        </div>
        <div className="col-info__img__container">
          <img className="col-info__img" src="/img/letter.png"/>
        </div>  
      </div>  
      <div className="col-form">
        <form className="feedbackForm" ref={form} onSubmit={sendEmail}>
            <input name="user_page" type="hidden" value={sourcePage} />
            <div className="feedbackForm__field">
              <label htmlFor="user_object">What's your headache? *</label>
              <input type="text" required placeholder="I miss information about Luos services..." name="user_object"/>
            </div> 
            <div className="feedbackForm__field">
              <label htmlFor="message">In more details</label>
              <textarea rows="9" name="message" placeholder="How should I configure my services in order to get my rocket off the ground ?"/>
            </div> 
            <div className="feedbackForm__field">
              <label htmlFor="user_name">Your Full Name</label>
              <input type="text" placeholder="John Doe" name="user_name"/>
            </div> 
            <div className="feedbackForm__field">
              <label htmlFor="user_email">Your Email * (to reach you with a solution)</label>
              <input required type="email" placeholder="john@luos.io" name="user_email"/>
            </div>
          <input type="submit" value="Send"/>
        </form>
        <div className="feedbackForm__response">
            <p id ="response"></p>
        </div>
      </div>
    </div>
  );
};