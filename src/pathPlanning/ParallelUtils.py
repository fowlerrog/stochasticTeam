# python imports
import asyncio
import typing

def background(f):
    """
    Decorates a function so it can be easily parallelized by asyncio
    
    Declaration:
    @background
    f(...):
    
    Usage:
    loop = asyncio.get_event_loop()
    looper = asyncio.gather(*[f(args) for args in argList])
    results = loop.run_until_complete(looper)
    """
    def wrapped(*args, **kwargs):
        return asyncio.get_event_loop().run_in_executor(None, f, *args, **kwargs)

    return wrapped

def asyncioParallelEval(f, elements, *args, **kwargs):
    """
	Evaluates [f(e, *args, **kwargs) for e in elements] in parallel
		best suited to I/O-bound processes
    """

    assert all(isinstance(e, typing.Hashable) for e in elements), 'Asyncio parallel loop requires hashable arguments'

	# Create parallel-friendly function
    @background
    def parallelF(i):
        return (i, f(elements[i], *args, **kwargs))

	# Create loop and run
    loop = asyncio.get_event_loop()
    looper = asyncio.gather(*[parallelF(i) for i in range(len(elements))])
    results = loop.run_until_complete(looper)

    # Reorder unordered results to match input
    output = [None] * len(results)
    for r in results:
        output[r[0]] = r[1]
    return output

# TODO convert this stuff to multiprocessing