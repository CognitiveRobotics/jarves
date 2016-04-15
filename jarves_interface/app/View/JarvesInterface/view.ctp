<!-- this auto-generates the connecting to ROS code -->
<?php echo $this->Rms->ros($environment['Rosbridge']['uri']); ?>


<script>
        var topic = new ROSLIB.Topic({
                ros : _ROS,
                name : '/echo',
                messageType : 'std_msgs/String'
                });

        var send_location = new ROSLIB.Service({
                ros : _ROS,
                name : '/office_destination',
                serviceType : 'jarves/Destination'
                });
</script>

<header class="special container">
        <span class="icon fa-android" aria-hidden="true"></span>
        <h2>Jarves</h2>
</header>

<!-- we need 'n' buttons to signify the office locations we have in our database -->

<section class="wrapper style4 container">
        <div class="content center">
                <button id="send1" class="button">Dr H Christensen</button>
                <script>
                        $('#send1').click(function() {
                                var request = new ROSLIB.ServiceRequest({
                                        destination : 'henrik'
                                        });
                                
                                send_location.callService(request, function(result) {
                                        console.log('Result :' + result.success);
                                        });
                        });
                </script>
                
                <br />
                <br />
                <button id="send2" class="button">Dr S Chernova</button>
                <script>
                        $('#send2').click(function() {
                                var request = new ROSLIB.ServiceRequest({
                                        destination : 'sonia'
                                        });
                                
                                send_location.callService(request, function(result) {
                                        console.log('Result :' + result.success);
                                        });
                        });
                </script>
        </div>
</section>
