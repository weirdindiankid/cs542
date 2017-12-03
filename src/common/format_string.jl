# Author: Youngjun Kim, youngjun@stanford.edu
# Data: 05/03/2014


# make a string from value by "%g" format
# it was implemented since Julia did not support "%g" format.
# this function can be obsolete if Julia introduces the format.

function outputGFormatString(v::Real)

    num_str = string(v)

    temp = split(num_str, 'e')

    digits = ""
    exponent = ""

    if length(temp) == 1
        digits = temp[1]
    elseif length(temp) == 2
        digits = temp[1]
        exponent = temp[2]
    else
        error("not a valid number")
    end

    b_dot = false
    sig_digit = 0

    i = 1
    for i = 1:length(digits)
        if digits[i] == '.'
            if sig_digit >= 6
                i = i - 1
                break
            end

            b_dot = true
        elseif isdigit(digits[i])
            if sig_digit != 0 || digits[i] != '0'
                sig_digit += 1

                if b_dot && sig_digit == 6
                    break
                end
            end
        elseif digits[i] == '-'
        else
            error("not a valid number")
        end
    end

    end_ind = i

    if b_dot
        for j = 1:i
            k = i - j + 1

            if digits[k] == '.'
                end_ind = k - 1
                break
            elseif digits[k] != '0'
                end_ind = k
                break
            end

        end
    end

    if exponent == ""
        return digits[1:end_ind]
    else
        return digits[1:end_ind] * "e" * exponent
    end
end


# validate output_g_format_string()
#
# validation step
# 1. generate a sample file without output_g_format_string()
# 2. generate a sample file with output_g_format_string()
# 3. compare values of two files

function validate_output_g_format_string(sample_filename_1, sample_filename_2)

    f1 = open(sample_filename_1, "r")
    f2 = open(sample_filename_2, "r")
    
    readline(f1)
    readline(f2)
    
    for line1 = eachline(f1)
        line2 = readline(f2)

        values1 = float(split(chomp(line1)))
        values2 = float(split(chomp(line2)))

        for i = 1:length(values1)
            if abs((values1[i] - values2[i]) / values2[i] * 100) > 0.01
                println("line 1: ", chomp(line1))
                println("line 2: ", chomp(line2))
                println("i: ", i)
                println("value 1: ", values1[i])
                println("value 2: ", values2[i])
                error("Invalid")
            end
        end
    end

    close(f1)
    close(f2)
end


