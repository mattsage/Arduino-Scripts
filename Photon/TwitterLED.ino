int ledpin = D0;
int ledpin2 = D7;
void setup()
{
// register the cloud function
Spark.function("tweet", twittermessage);
pinMode(ledpin, OUTPUT);
pinMode(ledpin2, OUTPUT);
 
}
 
void loop()
{
delay(5000);
digitalWrite(ledpin, LOW);
digitalWrite(ledpin2, LOW);
delay(5000);
}
 
// this function automagically gets called upon a tweet mention
int twittermessage(String ignore)
{
digitalWrite(ledpin, HIGH);
digitalWrite(ledpin2, HIGH);
delay(5400000); //Stay On for 5 seconds (90Mins = 5400000)
return(0);
}
