  .globl  maxofthree
  .text
maxofthree:
  mov     %rdi, %rax                # result (rax) initially holds x
  cmp     %rsi, %rax                # is x less than y?
  cmovl   %rsi, %rax                # if so, set result to y
  cmp     %rdx, %rax                # is max(x,y) less than z?
  cmovl   %rdx, %rax                # if so, set result to z
  ret                               # the max will be in eax
