import smtplib
from email.MIMEMultipart import MIMEMultipart
from email.MIMEText import MIMEText


def send_email(subj, text):
    '''
    WARNING: This may not be fully secure, use at your own risk
    '''
    msg = MIMEMultipart()
    msg['From'] = FROMADDR
    msg['To'] = TOADDR
    msg['Subject'] = subj

    body = text
    msg.attach(MIMEText(body, 'plain'))

    server = smtplib.SMTP('smtp.mail.yahoo.com', 587)
    server.starttls()
    server.login(FROMADDR, FROMADDR_PASSWORD)
    text = msg.as_string()
    server.sendmail(FROMADDR, TOADDR, text)
    server.quit()
