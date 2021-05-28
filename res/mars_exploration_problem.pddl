(define (problem mars_exploration_problem)
    (:domain mars_exploration)

    (:objects rover1 - rover
              p0000 p0001 p0510 p1005 p2505 p1314 p1020 p3020 p3030 - Point
              sample1 sample2 sample3 sample4 - sample)

    (:init (rover_at rover1 p0000)
           (sample_from sample1 p1005)
           (sample_from sample2 p2505)
           (sample_from sample3 p1020)
           (sample_from sample4 p3020))

    (:goal (and (sample_taken sample1)
                (sample_taken sample2)
                (sample_taken sample3)
                (sample_taken sample4)
                (picture_taken p1314)
                (picture_taken p0510)
                (picture_taken p2505)
                (picture_taken p1020)
                (picture_taken p3030)
                (communicated rover1 p2505)
                (communicated rover1 p3020)
                (rover_at rover1 p3030))))
